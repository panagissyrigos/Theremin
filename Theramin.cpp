#define NOMINMAX
#include <windows.h>
#include <windowsx.h>
#include <mmdeviceapi.h>
#include <audioclient.h>
#include <avrt.h>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <vector>
#include <string>

#pragma comment(lib,"Ole32.lib")
#pragma comment(lib,"Mmdevapi.lib")
#pragma comment(lib,"Avrt.lib")

// Simple HRESULT check macro
#define CHECKHR(hr) do { if (FAILED(hr)) { goto cleanup; } } while (0)

// Safe Release helper
template <class T>
void SafeRelease(T** ppT) { if (ppT && *ppT) { (*ppT)->Release(); *ppT = nullptr; } }

// ------------------------------
// Synth parameters and utilities
// ------------------------------

static constexpr float kSampleRate = 48000.0f;
static constexpr float kTwoPi = 6.28318530717958647692f;
static constexpr float kMinHz = 100.0f;
static constexpr float kMaxHz = 2000.0f;

struct SynthParams {
    std::atomic<float> targetHz{ 440.0f };
    std::atomic<float> targetGain{ 0.0f };   // 0..1
    std::atomic<int>   mode{ 1 };            // 1..4
    std::atomic<bool>  mute{ false };
    std::atomic<float> vibratoDepth{ 0.0f }; // 0..1 (depth scaled in synth)
};

struct SynthState {
    float phaseA = 0.0f; // main osc
    float phaseB = 0.0f; // mod osc
    float smoothHz = 440.0f;
    float smoothGain = 0.0f;
    float vibratoPhase = 0.0f;
    float delayL = 0.0f, delayR = 0.0f; // minimal stereo decorrelation
    std::vector<float> delayBufL, delayBufR;
    size_t delayIndex = 0;
};

static inline float fast_tanhf(float x) {
    // Rational tanh approximation (sufficient for gentle waveshaping)
    // tanh(x) ~ x * (27 + x^2) / (27 + 9*x^2)
    const float x2 = x * x;
    return x * (27.0f + x2) / (27.0f + 9.0f * x2);
}

// PolyBLEP-free soft saw/tri hybrid (simple, slightly band-limited by saturation)
static inline float soft_saw(float phase) {
    // Map phase to (-1..1) saw, then gently saturate
    float s = (phase / kTwoPi) * 2.0f - 1.0f; // -1..1 ramp
    return fast_tanhf(0.8f * s);
}

static inline float soft_tri(float phase) {
    float tri = 2.0f * fabsf((phase / kTwoPi) - 0.5f) - 1.0f;
    return fast_tanhf(0.8f * tri);
}

static inline float sine(float phase) {
    return sinf(phase);
}

static inline float white_noise() {
    // Simple LCG-based white noise; deterministic enough for demo
    static uint32_t seed = 0x12345678u;
    seed = 1664525u * seed + 1013904223u;
    const float u = (seed & 0x00FFFFFFu) / 16777216.0f; // [0,1)
    return 2.0f * u - 1.0f; // [-1,1]
}

// One-pole smoother (slew) for frequency and gain
static inline float smooth_step(float current, float target, float coeff) {
    return current + coeff * (target - current);
}

// Map mouse X (0..W) to logarithmic frequency between kMinHz and kMaxHz
static inline float map_x_to_hz(int x, int width) {
    if (width <= 0) return 440.0f;
    float nx = std::max(0.0f, std::min(1.0f, x / float(width)));
    // Log mapping: Hz = Min * (Max/Min)^nx
    float ratio = kMaxHz / kMinHz;
    return kMinHz * powf(ratio, nx);
}

// Map mouse Y (0..H) to gain (top loud, bottom quiet); clamp 0..1
static inline float map_y_to_gain(int y, int height) {
    if (height <= 0) return 0.0f;
    float ny = std::max(0.0f, std::min(1.0f, y / float(height)));
    return 1.0f - ny; // invert (top loud)
}

// ------------------------------
// WASAPI infrastructure
// ------------------------------

struct WasapiContext {
    IMMDeviceEnumerator* pEnum = nullptr;
    IMMDevice* pDev = nullptr;
    IAudioClient* pCli = nullptr;
    IAudioRenderClient* pRen = nullptr;
    HANDLE               hEvent = nullptr;
    WAVEFORMATEX* pMixFmt = nullptr;
    UINT32               bufferFrames = 0;
    HANDLE               hAudioThread = nullptr;
    HANDLE               hAvrt = nullptr;
    std::atomic<bool>    running{ false };
    bool                 coInit = false;
};

// ------------------------------
// Global app state
// ------------------------------

static SynthParams gParams;
static WasapiContext gWASAPI;
static SynthState gSynth;
static HWND gHWND = nullptr;

// ------------------------------
// Audio render thread
// ------------------------------

DWORD WINAPI AudioThreadMain(LPVOID) {
    // Boost thread priority for audio
    DWORD taskIdx = 0;
    gWASAPI.hAvrt = AvSetMmThreadCharacteristicsW(L"Pro Audio", &taskIdx);

    gWASAPI.running.store(true);
    const float sampleRate = float(gWASAPI.pMixFmt->nSamplesPerSec);
    const int   channels = int(gWASAPI.pMixFmt->nChannels);
    const float dt = 1.0f / sampleRate;

    // Delay line for subtle stereo decorrelation
    const size_t delaySamples = size_t(sampleRate * 0.012f); // 12 ms
    gSynth.delayBufL.assign(delaySamples, 0.0f);
    gSynth.delayBufR.assign(delaySamples, 0.0f);
    gSynth.delayIndex = 0;

    // Smooth coefficients (fast but safe)
    const float hzSmoothCoeff = 0.05f;  // frequency slew
    const float gainSmoothCoeff = 0.075f; // amplitude slew

    // Start
    HRESULT hr = gWASAPI.pCli->Start();
    if (FAILED(hr)) goto done;

    while (gWASAPI.running.load()) {
        DWORD waitRes = WaitForSingleObject(gWASAPI.hEvent, 5 /*ms timeout*/);
        if (waitRes != WAIT_OBJECT_0) continue;

        UINT32 padding = 0;
        hr = gWASAPI.pCli->GetCurrentPadding(&padding);
        if (FAILED(hr)) break;

        UINT32 framesToWrite = gWASAPI.bufferFrames - padding;
        if (framesToWrite == 0) continue;

        BYTE* pData = nullptr;
        hr = gWASAPI.pRen->GetBuffer(framesToWrite, &pData);
        if (FAILED(hr) || !pData) break;

        float* out = reinterpret_cast<float*>(pData);

        for (UINT32 i = 0; i < framesToWrite; ++i) {
            // Read targets
            float tgtHz = gParams.targetHz.load();
            float tgtGain = gParams.targetGain.load();
            bool  mute = gParams.mute.load();
            int   mode = gParams.mode.load();
            float vibAmt = gParams.vibratoDepth.load();

            // Smooth
            gSynth.smoothHz = smooth_step(gSynth.smoothHz, tgtHz, hzSmoothCoeff);
            gSynth.smoothGain = smooth_step(gSynth.smoothGain, tgtGain, gainSmoothCoeff);

            // Vibrato (5.5 Hz)
            gSynth.vibratoPhase += kTwoPi * 5.5f * dt;
            if (gSynth.vibratoPhase >= kTwoPi) gSynth.vibratoPhase -= kTwoPi;
            float vibrato = (vibAmt > 0.0f) ? 0.01f * vibAmt * sinf(gSynth.vibratoPhase) : 0.0f;

            float hz = gSynth.smoothHz * (1.0f + vibrato);
            float incA = kTwoPi * hz * dt;
            float incB = kTwoPi * (hz * 1.997f) * dt; // mod osc ~2x main

            // Advance phases
            gSynth.phaseA += incA;
            if (gSynth.phaseA >= kTwoPi) gSynth.phaseA -= kTwoPi;
            gSynth.phaseB += incB;
            if (gSynth.phaseB >= kTwoPi) gSynth.phaseB -= kTwoPi;

            // Base tones
            float aSine = sine(gSynth.phaseA);
            float bSine = sine(gSynth.phaseB);
            float sample = 0.0f;

            switch (mode) {
            case 1: { // pure sine
                sample = aSine;
            } break;
            case 2: { // sine + ring modulation
                float ring = aSine * bSine;         // sidebands
                sample = 0.70f * aSine + 0.45f * ring;
            } break;
            case 3: { // airy: sine + noise + gentle saturation
                float n = 0.25f * white_noise();
                float pre = 0.85f * aSine + n;
                sample = fast_tanhf(pre);           // soft saturation
            } break;
            case 4: { // soft saw/tri hybrid
                float s = soft_saw(gSynth.phaseA);
                float t = soft_tri(gSynth.phaseA);
                sample = 0.6f * s + 0.4f * t;
            } break;
            default: sample = aSine; break;
            }

            // Apply amplitude and mute
            float gain = mute ? 0.0f : gSynth.smoothGain;
            float dryL = sample * gain;
            float dryR = sample * gain;

            // Minimal stereo decorrelation via short delay & crossfeed
            size_t di = gSynth.delayIndex;
            float dL = gSynth.delayBufL[di];
            float dR = gSynth.delayBufR[di];
            gSynth.delayBufL[di] = 0.85f * dL + 0.15f * dryL;
            gSynth.delayBufR[di] = 0.85f * dR + 0.15f * dryR;
            gSynth.delayIndex = (di + 1) % gSynth.delayBufL.size();

            float outL = 0.85f * dryL + 0.15f * dR;
            float outR = 0.85f * dryR + 0.15f * dL;

            // Write interleaved stereo float
            out[i * channels + 0] = outL;
            if (channels > 1) out[i * channels + 1] = outR;
        }

        hr = gWASAPI.pRen->ReleaseBuffer(framesToWrite, 0);
        if (FAILED(hr)) break;
    }

done:
    if (gWASAPI.pCli) gWASAPI.pCli->Stop();
    if (gWASAPI.hAvrt) { AvRevertMmThreadCharacteristics(gWASAPI.hAvrt); gWASAPI.hAvrt = nullptr; }
    return 0;
}

// ------------------------------
// Win32 window and input
// ------------------------------

LRESULT CALLBACK WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    switch (msg) {
    case WM_DESTROY:
        gWASAPI.running.store(false);
        PostQuitMessage(0);
        return 0;
    case WM_MOUSEMOVE: {
        int x = GET_X_LPARAM(lParam);
        int y = GET_Y_LPARAM(lParam);
        RECT rc{}; GetClientRect(hWnd, &rc);
        float hz = map_x_to_hz(x, rc.right - rc.left);
        float gain = map_y_to_gain(y, rc.bottom - rc.top);
        gParams.targetHz.store(hz);
        gParams.targetGain.store(gain);
        // Shift increases vibrato depth
        bool shift = (GetKeyState(VK_SHIFT) & 0x8000) != 0;
        gParams.vibratoDepth.store(shift ? 1.0f : 0.0f);
        return 0;
    }
    case WM_KEYDOWN: {
        switch (wParam) {
        case '1': gParams.mode.store(1); break;
        case '2': gParams.mode.store(2); break;
        case '3': gParams.mode.store(3); break;
        case '4': gParams.mode.store(4); break;
        case VK_SPACE: {
            bool m = gParams.mute.load();
            gParams.mute.store(!m);
        } break;
        case VK_ESCAPE:
            DestroyWindow(hWnd);
            break;
        }
        return 0;
    }
    case WM_LBUTTONDOWN:
    case WM_RBUTTONDOWN:
    case WM_MBUTTONDOWN:
        SetCapture(hWnd);
        return 0;
    case WM_LBUTTONUP:
    case WM_RBUTTONUP:
    case WM_MBUTTONUP:
        ReleaseCapture();
        return 0;
    }
    return DefWindowProc(hWnd, msg, wParam, lParam);
}

// ------------------------------
// WASAPI setup/teardown
// ------------------------------

// Replace your ShutdownWASAPI with this:
void ShutdownWASAPI() {
    gWASAPI.running.store(false);

    if (gWASAPI.hAudioThread) {
        WaitForSingleObject(gWASAPI.hAudioThread, 2000);
        CloseHandle(gWASAPI.hAudioThread);
        gWASAPI.hAudioThread = nullptr;
    }
    if (gWASAPI.hEvent) {
        CloseHandle(gWASAPI.hEvent);
        gWASAPI.hEvent = nullptr;
    }

    SafeRelease(&gWASAPI.pRen);
    SafeRelease(&gWASAPI.pCli);
    SafeRelease(&gWASAPI.pDev);
    SafeRelease(&gWASAPI.pEnum);

    if (gWASAPI.pMixFmt) {
        CoTaskMemFree(gWASAPI.pMixFmt);
        gWASAPI.pMixFmt = nullptr;
    }
    if (gWASAPI.coInit) {
        CoUninitialize();
        gWASAPI.coInit = false;
    }
}

// Replace your InitWASAPI with this:
bool InitWASAPI(HWND) {
    // COM init
    HRESULT hr = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
    if (FAILED(hr)) return false;
    gWASAPI.coInit = true;

    // Device enumerator
    hr = CoCreateInstance(__uuidof(MMDeviceEnumerator), nullptr, CLSCTX_ALL,
        __uuidof(IMMDeviceEnumerator), (void**)&gWASAPI.pEnum);
    if (FAILED(hr)) { ShutdownWASAPI(); return false; }

    // Default render endpoint
    hr = gWASAPI.pEnum->GetDefaultAudioEndpoint(eRender, eConsole, &gWASAPI.pDev);
    if (FAILED(hr)) { ShutdownWASAPI(); return false; }

    // Audio client
    hr = gWASAPI.pDev->Activate(__uuidof(IAudioClient), CLSCTX_ALL, nullptr, (void**)&gWASAPI.pCli);
    if (FAILED(hr)) { ShutdownWASAPI(); return false; }

    // Mix format (shared-mode format)
    hr = gWASAPI.pCli->GetMixFormat(&gWASAPI.pMixFmt);
    if (FAILED(hr) || !gWASAPI.pMixFmt) { ShutdownWASAPI(); return false; }

    // Initialize shared, event-driven stream
    REFERENCE_TIME hnsBufferDuration = 20 * 10000; // 20 ms
    hr = gWASAPI.pCli->Initialize(
        AUDCLNT_SHAREMODE_SHARED,
        AUDCLNT_STREAMFLAGS_EVENTCALLBACK | AUDCLNT_STREAMFLAGS_RATEADJUST,
        hnsBufferDuration,
        0,
        gWASAPI.pMixFmt,
        nullptr);
    if (FAILED(hr)) { ShutdownWASAPI(); return false; }

    // Buffer size
    hr = gWASAPI.pCli->GetBufferSize(&gWASAPI.bufferFrames);
    if (FAILED(hr) || gWASAPI.bufferFrames == 0) { ShutdownWASAPI(); return false; }

    // Event
    gWASAPI.hEvent = CreateEventW(nullptr, FALSE, FALSE, L"WASAPIEvent");
    if (!gWASAPI.hEvent) { ShutdownWASAPI(); return false; }

    hr = gWASAPI.pCli->SetEventHandle(gWASAPI.hEvent);
    if (FAILED(hr)) { ShutdownWASAPI(); return false; }

    // Render client
    hr = gWASAPI.pCli->GetService(__uuidof(IAudioRenderClient), (void**)&gWASAPI.pRen);
    if (FAILED(hr) || !gWASAPI.pRen) { ShutdownWASAPI(); return false; }

    // Pre-roll silence
    BYTE* pData = nullptr;
    hr = gWASAPI.pRen->GetBuffer(gWASAPI.bufferFrames, &pData);
    if (FAILED(hr) || !pData) { ShutdownWASAPI(); return false; }

    memset(pData, 0, gWASAPI.bufferFrames * gWASAPI.pMixFmt->nBlockAlign);
    hr = gWASAPI.pRen->ReleaseBuffer(gWASAPI.bufferFrames, 0);
    if (FAILED(hr)) { ShutdownWASAPI(); return false; }

    // Audio thread
    gWASAPI.hAudioThread = CreateThread(nullptr, 0, AudioThreadMain, nullptr, 0, nullptr);
    if (!gWASAPI.hAudioThread) { ShutdownWASAPI(); return false; }

    return true;
}


// ------------------------------
// WinMain
// ------------------------------

int APIENTRY wWinMain(HINSTANCE hInst, HINSTANCE, LPWSTR, int nCmdShow) {
    // Window class
    WNDCLASSW wc{};
    wc.lpfnWndProc = WndProc;
    wc.hInstance = hInst;
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.lpszClassName = L"ThereminWindowClass";
    RegisterClassW(&wc);

    // Window
    gHWND = CreateWindowExW(0, wc.lpszClassName, L"Theremin (WASAPI) - Mouse X=Pitch, Y=Volume | 1-4 Modes | Shift Vibrato | Space Mute",
        WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, CW_USEDEFAULT, 900, 300,
        nullptr, nullptr, hInst, nullptr);
    if (!gHWND) return 0;
    ShowWindow(gHWND, nCmdShow);

    // Init audio
    if (!InitWASAPI(gHWND)) {
        MessageBoxW(gHWND, L"Failed to initialize WASAPI.", L"Error", MB_OK | MB_ICONERROR);
        DestroyWindow(gHWND);
        return 0;
    }

    // Message loop
    MSG msg{};
    while (GetMessageW(&msg, nullptr, 0, 0) > 0) {
        TranslateMessage(&msg);
        DispatchMessageW(&msg);
    }

    ShutdownWASAPI();
    return 0;
}
