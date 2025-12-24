#include <distingnt/api.h>
#include <new>

// Algorithm state
struct BiquadState {
    float x1, x2, y1, y2;
};

struct TapeState {
    BiquadState headBump;
    BiquadState hfRollOff;
    float lastPreSat;
};

struct MrFreezeAlgorithm : public _NT_algorithm {
    float* audioBuffer;

    // Parameters
    int dryInL, dryInR;
    int fbInL, fbInR;
    int clockIn;
    int outL, outR;
    bool freeze;
    int division;
    int triplet;
    bool dotted;
    float crossfade;
    bool pingPong;
    int sync;
    float tempo;
    float feedback;
    float base, width;
    int bitDepth;
    float tapeSat;

    // State
    uint32_t bufferSize;
    uint32_t writeHead;
    float lpfStateL[2], lpfStateR[2]; // Simple state for filters
    float hpfStateL[2], hpfStateR[2];

    // Clock State
    float lastClockSample;
    uint32_t framesSinceLastClock;
    float detectedBpm;

    // MIDI Clock State
    uint32_t framesSinceLastMidiClock;
    float detectedMidiBpm;

    // Limiter State
    float limiterEnvelope;

    // Loop State
    uint32_t loopPhase;
    uint32_t freezeStartPos;
    bool wasFrozen;
    uint32_t currentDelayLen;

    // Tape State
    TapeState tapeL, tapeR;
    float lastSampleRate;
    float lastTapeSat;
    float headBumpCoeffs[5];
    float hfRollOffCoeffs[5];
};

// Parameter indices
enum {
    kParam_DryInL,
    kParam_DryInR,
    kParam_FBInL,
    kParam_FBInR,
    kParam_ClockIn,
    kParam_OutL,
    kParam_OutR,
    kParam_Freeze,
    kParam_Division,
    kParam_Triplet,
    kParam_Dotted,
    kParam_Crossfade,
    kParam_PingPong,
    kParam_Sync,
    kParam_Tempo,
    kParam_Feedback,
    kParam_Base,
    kParam_Width,
    kParam_BitDepth,
    kParam_LoFiSR,
    kParam_TapeSat,
    kNumParameters
};

// Specifications
enum {
    kSpec_MaxDelayTime,
    kNumSpecifications
};

// Plugin GUID
static const uint32_t kGuid = NT_MULTICHAR('M', 'r', 'F', 'z');

// Enum Strings
static const char* const s_offOn[] = { "Off", "On", nullptr };
static const char* const s_divs[] = { "1/64", "1/32", "1/16", "1/8", "1/4", "1/2", "1/1", nullptr };
static const char* const s_triplet[] = { "Off", "x3", "/3", nullptr };
static const char* const s_dotted[] = { "Off", "x1.5", nullptr };
static const char* const s_sync[] = { "Int", "Clock", "MIDI", nullptr };

// Parameter definitions
static const _NT_parameter parameters[kNumParameters] = {
    // Page 1: Routing & I/O
    [kParam_DryInL] = { .name = "Dry In L", .min = 1, .max = 28, .def = 1, .unit = kNT_unitAudioInput },
    [kParam_DryInR] = { .name = "Dry In R", .min = 0, .max = 28, .def = 2, .unit = kNT_unitAudioInput },
    [kParam_FBInL]  = { .name = "FB In L",  .min = 0, .max = 28, .def = 0, .unit = kNT_unitAudioInput },
    [kParam_FBInR]  = { .name = "FB In R",  .min = 0, .max = 28, .def = 0, .unit = kNT_unitAudioInput },
    [kParam_ClockIn]= { .name = "Clock In", .min = 0, .max = 28, .def = 0, .unit = kNT_unitCvInput },
    [kParam_OutL]   = { .name = "Out L",    .min = 1, .max = 28, .def = 13, .unit = kNT_unitAudioOutput },
    [kParam_OutR]   = { .name = "Out R",    .min = 1, .max = 28, .def = 14, .unit = kNT_unitAudioOutput },

    // Page 2: Performance & Timing
    [kParam_Freeze]   = { .name = "Freeze",    .min = 0, .max = 1, .def = 0, .unit = kNT_unitEnum, .enumStrings = s_offOn },
    [kParam_Division] = { .name = "Division",  .min = 0, .max = 6, .def = 4, .unit = kNT_unitEnum, .enumStrings = s_divs },
    [kParam_Triplet]  = { .name = "Triplet",   .min = 0, .max = 2, .def = 0, .unit = kNT_unitEnum, .enumStrings = s_triplet },
    [kParam_Dotted]   = { .name = "Dotted",    .min = 0, .max = 1, .def = 0, .unit = kNT_unitEnum, .enumStrings = s_dotted },
    [kParam_Crossfade]= { .name = "Crossfade %",.min = 0, .max = 50, .def = 10, .unit = kNT_unitPercent },
    [kParam_PingPong] = { .name = "Ping Pong", .min = 0, .max = 1, .def = 0, .unit = kNT_unitEnum, .enumStrings = s_offOn },
    [kParam_Sync]     = { .name = "Sync",      .min = 0, .max = 2, .def = 1, .unit = kNT_unitEnum, .enumStrings = s_sync },
    [kParam_Tempo]    = { .name = "Tempo",     .min = 30, .max = 300, .def = 120, .unit = kNT_unitBPM },

    // Page 3: Character & Filter
    [kParam_Feedback] = { .name = "Feedback", .min = 0, .max = 100, .def = 50, .unit = kNT_unitPercent },
    [kParam_Base]     = { .name = "Base",     .min = 0, .max = 127, .def = 0, .unit = kNT_unitNone },
    [kParam_Width]    = { .name = "Width",    .min = 0, .max = 127, .def = 127, .unit = kNT_unitNone },
    [kParam_BitDepth] = { .name = "Bit Depth",.min = 1, .max = 24, .def = 24, .unit = kNT_unitNone },
    [kParam_LoFiSR]   = { .name = "Lo-Fi SR", .min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent },
    [kParam_TapeSat]  = { .name = "Tape Sat", .min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent },
};

// Parameter pages
static const uint8_t page1[] = { kParam_DryInL, kParam_DryInR, kParam_FBInL, kParam_FBInR, kParam_ClockIn, kParam_OutL, kParam_OutR };
static const uint8_t page2[] = { kParam_Freeze, kParam_Division, kParam_Triplet, kParam_Dotted, kParam_Crossfade, kParam_PingPong, kParam_Sync, kParam_Tempo };
static const uint8_t page3[] = { kParam_Feedback, kParam_Base, kParam_Width, kParam_BitDepth, kParam_LoFiSR, kParam_TapeSat };

static const _NT_parameterPage pages[] = {
    { .name = "Routing & I/O", .numParams = ARRAY_SIZE(page1), .params = page1 },
    { .name = "Performance",   .numParams = ARRAY_SIZE(page2), .params = page2 },
    { .name = "Character",     .numParams = ARRAY_SIZE(page3), .params = page3 },
};
static const _NT_parameterPages parameterPages = {
    .numPages = ARRAY_SIZE(pages),
    .pages = pages,
};

// Specifications
static const _NT_specification specificationsInfo[] = {
    [kSpec_MaxDelayTime] = { .name = "Max Delay Time", .min = 1, .max = 20, .def = 10, .type = kNT_typeSeconds },
};

// --- Math Helpers ---

static inline float my_abs(float x) { return (x < 0.0f) ? -x : x; }
static inline float my_max(float a, float b) { return (a > b) ? a : b; }
static inline float my_min(float a, float b) { return (a < b) ? a : b; }

// Sine Table (0 to pi/2) for Equal Power Crossfade
// 32 segments (33 points)
static const float kSineTable[33] = {
    0.000000f, 0.049068f, 0.098017f, 0.146730f, 0.195090f, 0.242980f, 0.290285f, 0.336890f,
    0.382683f, 0.427555f, 0.471397f, 0.514103f, 0.555570f, 0.595699f, 0.634393f, 0.671559f,
    0.707107f, 0.740951f, 0.773010f, 0.803208f, 0.831470f, 0.857729f, 0.881921f, 0.903989f,
    0.923880f, 0.941544f, 0.956940f, 0.970031f, 0.980785f, 0.989177f, 0.995185f, 0.998800f,
    1.000000f
};

static inline float getSineInterp(float phase01) {
    if (phase01 <= 0.0f) return 0.0f;
    if (phase01 >= 1.0f) return 1.0f;
    float idxF = phase01 * 32.0f;
    int idx = (int)idxF;
    float frac = idxF - (float)idx;
    return kSineTable[idx] * (1.0f - frac) + kSineTable[idx + 1] * frac;
}

// Helper: Biquad Process
static inline float processBiquad(float x, BiquadState& s, const float* c) {
    // y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
    float y = c[0]*x + c[1]*s.x1 + c[2]*s.x2 - c[3]*s.y1 - c[4]*s.y2 + 1.0e-18f;
    s.x2 = s.x1; s.x1 = x;
    s.y2 = s.y1; s.y1 = y;
    return y;
}

// Helper: Saturation Non-linearity
static inline float saturate(float x) {
    x += 0.05f; // Bias
    if (x > 1.0f) x = 0.6666667f; 
    else if (x < -1.0f) x = -0.6666667f; 
    else x = x - (x * x * x * 0.3333333f);
    x -= 0.05f; // Remove Bias
    return x;
}

// Helper: Tape Saturation Chain
static inline float processTape(float x, float drive, TapeState& s, const float* hbCoeffs, const float* hfCoeffs) {
    float in = x * drive;
    in = processBiquad(in, s.headBump, hbCoeffs);
    
    // 2x Oversampling
    float mid = 0.5f * (in + s.lastPreSat);
    float out = 0.5f * (saturate(mid) + saturate(in));
    s.lastPreSat = in;

    out = processBiquad(out, s.hfRollOff, hfCoeffs);
    return out;
}

// Helper: Bit Crusher
static inline float bitCrush(float x, int depth) {
    if (depth >= 24) return x;
    float scale = (float)(1 << depth);
    return (float)((int)(x * scale)) / scale;
}

// Helper: Simple 1-pole Filter
static inline float processFilter(float input, float& state, float coeff, bool isHPF) {
    // y[n] = y[n-1] + coeff * (x[n] - y[n-1])
    state += coeff * (input - state);
    if (isHPF) return input - state;
    return state;
}

// --- Core Functions ---

extern "C" {

void calculateRequirements(_NT_algorithmRequirements& req, const int32_t* specifications) {
    req.numParameters = kNumParameters;
    req.sram = sizeof(MrFreezeAlgorithm);
    
    // Calculate DRAM based on Max Delay Time (spec 0)
    // Assuming 96kHz sample rate for safety, stereo (2 channels), float (4 bytes)
    int maxTime = specifications ? specifications[kSpec_MaxDelayTime] : 10;
    req.dram = maxTime * 96000 * 2 * sizeof(float);
    
    req.dtc = 0;
    req.itc = 0;
}

_NT_algorithm* construct(const _NT_algorithmMemoryPtrs& ptrs, const _NT_algorithmRequirements& req, const int32_t* specifications) {
    MrFreezeAlgorithm* alg = new (ptrs.sram) MrFreezeAlgorithm();
    alg->parameters = parameters;
    alg->parameterPages = &parameterPages;
    alg->audioBuffer = (float*)ptrs.dram;
    
    // Initialize state
    int maxTime = specifications ? specifications[kSpec_MaxDelayTime] : 10;
    alg->bufferSize = maxTime * 96000; // Stereo frames
    alg->writeHead = 0;
    
    // Initialize defaults
    alg->feedback = 0.5f;
    alg->freeze = false;
    alg->bitDepth = 24;
    alg->width = 1.0f;
    alg->dryInL = 1;
    alg->dryInR = 2;
    alg->outL = 13;
    alg->outR = 14;
    alg->sync = 1;
    alg->division = 4;
    alg->tempo = 120.0f;
    alg->lastClockSample = 0.0f;
    alg->framesSinceLastClock = 0;
    alg->detectedBpm = 120.0f;
    alg->framesSinceLastMidiClock = 0;
    alg->detectedMidiBpm = 120.0f;
    alg->limiterEnvelope = 0.0f;
    alg->loopPhase = 0;
    alg->freezeStartPos = 0;
    alg->wasFrozen = false;
    alg->currentDelayLen = 0;
    
    // Init Tape State
    alg->tapeL = {};
    alg->tapeR = {};
    alg->lastSampleRate = 0.0f;
    alg->lastTapeSat = -1.0f;
    // Coeffs will be calc'd in step
    
    return (_NT_algorithm*)alg;
}

void parameterChanged(_NT_algorithm* self, int p) {
    MrFreezeAlgorithm* pThis = (MrFreezeAlgorithm*)self;
    
    // Cache frequently accessed parameters
    switch (p) {
        case kParam_DryInL: pThis->dryInL = (int)self->v[p]; break;
        case kParam_DryInR: pThis->dryInR = (int)self->v[p]; break;
        case kParam_FBInL:  pThis->fbInL = (int)self->v[p]; break;
        case kParam_FBInR:  pThis->fbInR = (int)self->v[p]; break;
        case kParam_ClockIn:pThis->clockIn = (int)self->v[p]; break;
        case kParam_OutL:   pThis->outL = (int)self->v[p]; break;
        case kParam_OutR:   pThis->outR = (int)self->v[p]; break;
        case kParam_Freeze: pThis->freeze = (self->v[p] > 0.5f); break;
        case kParam_Division: pThis->division = (int)self->v[p]; break;
        case kParam_Triplet: pThis->triplet = (int)self->v[p]; break;
        case kParam_Dotted: pThis->dotted = (self->v[p] > 0.5f); break;
        case kParam_Feedback: pThis->feedback = self->v[p] / 100.0f; break;
        case kParam_PingPong: pThis->pingPong = (self->v[p] > 0.5f); break;
        case kParam_Sync:   pThis->sync = (int)self->v[p]; break;
        case kParam_Tempo:  pThis->tempo = (float)self->v[p]; break;
        case kParam_BitDepth: pThis->bitDepth = (int)self->v[p]; break;
        case kParam_TapeSat: pThis->tapeSat = self->v[p] / 100.0f; break;
        case kParam_Base: pThis->base = self->v[p] / 127.0f; break;
        case kParam_Width: pThis->width = self->v[p] / 127.0f; break;
        default: break;
    }
}

void midiRealtime(_NT_algorithm* self, uint8_t byte) {
    MrFreezeAlgorithm* pThis = (MrFreezeAlgorithm*)self;
    if (byte == 0xF8) { // Timing Clock
        if (pThis->framesSinceLastMidiClock > 0) {
            float seconds = pThis->framesSinceLastMidiClock / (float)NT_globals.sampleRate;
            if (seconds > 0.001f) {
                // 24 pulses per quarter note
                float newBpm = 60.0f / (seconds * 24.0f);
                // Simple smoothing
                pThis->detectedMidiBpm = pThis->detectedMidiBpm * 0.9f + newBpm * 0.1f;
            }
        }
        pThis->framesSinceLastMidiClock = 0;
    }
}

void step(_NT_algorithm* self, float* busFrames, int numFramesBy4) {
    MrFreezeAlgorithm* pThis = (MrFreezeAlgorithm*)self;
    int numFrames = numFramesBy4 * 4;

    // Pointers to IO
    const float* dryL = busFrames + (pThis->dryInL - 1) * numFrames;
    const float* dryR = (pThis->dryInR > 0) ? busFrames + (pThis->dryInR - 1) * numFrames : dryL;
    
    float* outL = busFrames + (pThis->outL - 1) * numFrames;
    float* outR = busFrames + (pThis->outR - 1) * numFrames;

    // Clock Input Pointer
    const float* clk = (pThis->clockIn > 0) ? busFrames + (pThis->clockIn - 1) * numFrames : nullptr;

    // Feedback Input Pointers
    const float* fbInL = (pThis->fbInL > 0) ? busFrames + (pThis->fbInL - 1) * numFrames : nullptr;
    const float* fbInR = (pThis->fbInR > 0) ? busFrames + (pThis->fbInR - 1) * numFrames : nullptr;

    // Filter Coeffs (Simple mapping)
    float hpfCoeff = pThis->base * 0.1f; 
    float lpfCoeff = my_min(1.0f, (pThis->base + pThis->width) * 0.1f + 0.01f);

    // Update Tape Coeffs if SR changed
    float sr = (float)NT_globals.sampleRate;
    if (sr != pThis->lastSampleRate || pThis->tapeSat != pThis->lastTapeSat) {
        pThis->lastSampleRate = sr;
        pThis->lastTapeSat = pThis->tapeSat;
        
        // 1. Head Bump: 80Hz, Q=0.707, +3dB
        float phaseHB = 4.0f * 80.0f / sr;
        float snHB = getSineInterp(phaseHB);
        float csHB = getSineInterp(1.0f - phaseHB);
        float alphaHB = snHB / (2.0f * 0.707f);
        float A = 1.0f + pThis->tapeSat * 0.1885f; // Scale Gain from 0dB (1.0) to +3dB (1.1885)
        float a0HB = 1.0f + alphaHB / A;
        pThis->headBumpCoeffs[0] = (1.0f + alphaHB * A) / a0HB;
        pThis->headBumpCoeffs[1] = (-2.0f * csHB) / a0HB;
        pThis->headBumpCoeffs[2] = (1.0f - alphaHB * A) / a0HB;
        pThis->headBumpCoeffs[3] = (-2.0f * csHB) / a0HB;
        pThis->headBumpCoeffs[4] = (1.0f - alphaHB / A) / a0HB;

        // 2. HF Roll-off: 12000Hz, Q=0.707, LPF
        float phaseHF = 4.0f * 12000.0f / sr;
        float snHF = getSineInterp(phaseHF);
        float csHF = getSineInterp(1.0f - phaseHF);
        float alphaHF = snHF / (2.0f * 0.707f);
        float a0HF = 1.0f + alphaHF;
        float oneMinusCs = 1.0f - csHF;
        pThis->hfRollOffCoeffs[0] = (oneMinusCs * 0.5f) / a0HF;
        pThis->hfRollOffCoeffs[1] = oneMinusCs / a0HF;
        pThis->hfRollOffCoeffs[2] = (oneMinusCs * 0.5f) / a0HF;
        pThis->hfRollOffCoeffs[3] = (-2.0f * csHF) / a0HF;
        pThis->hfRollOffCoeffs[4] = (1.0f - alphaHF) / a0HF;
    }

    // Limiter constants
    // exp(-1 / (SR * 0.05)) approx 1 - 1/(SR*0.05)
    float releaseFactor = 1.0f - (1.0f / (NT_globals.sampleRate * 0.05f));
    float threshold = 0.9885f; // -0.1 dB

    // Determine BPM
    float bpm = 120.0f;
    if (pThis->sync == 1 && clk) { // Sync: Clock
        // Simple Clock Detector
        for(int i=0; i<numFrames; ++i) {
            float s = clk[i];
            if (s > 2.0f && pThis->lastClockSample <= 2.0f) { // Rising edge
                if (pThis->framesSinceLastClock > 0) {
                    float seconds = pThis->framesSinceLastClock / (float)NT_globals.sampleRate;
                    if (seconds > 0.001f) {
                        pThis->detectedBpm = 60.0f / seconds;
                        // Handle 24ppqn or other dividers here if necessary, assuming 1/4 note triggers for now
                    }
                }
                pThis->framesSinceLastClock = 0;
            }
            pThis->framesSinceLastClock++;
            pThis->lastClockSample = s;
        }
        bpm = pThis->detectedBpm;
    } else if (pThis->sync == 2) { // Sync: MIDI
        bpm = pThis->detectedMidiBpm;
    } else {
        // Sync: Int
        bpm = pThis->tempo;
    }

    // Calculate Delay Time in Samples
    // Division map: 0=1/64, 1=1/32, 2=1/16, 3=1/8, 4=1/4, 5=1/2, 6=1/1
    float beats = 1.0f;
    switch(pThis->division) {
        case 0: beats = 0.0625f; break;
        case 1: beats = 0.125f; break;
        case 2: beats = 0.25f; break;
        case 3: beats = 0.5f; break;
        case 4: beats = 1.0f; break;
        case 5: beats = 2.0f; break;
        case 6: beats = 4.0f; break;
    }
    // Triplet: 0=Off, 1=x3 (fast), 2=/3 (slow)
    if (pThis->triplet == 1) beats /= 3.0f;
    else if (pThis->triplet == 2) beats *= 3.0f;
    // Dotted
    if (pThis->dotted) beats *= 1.5f;

    float samplesPerBeat = (NT_globals.sampleRate * 60.0f) / bpm;
    uint32_t delayLen = (uint32_t)(samplesPerBeat * beats);
    if (delayLen < 1) delayLen = 1;
    if (delayLen > pThis->bufferSize) delayLen = pThis->bufferSize;
    pThis->currentDelayLen = delayLen;

    // Handle Freeze Transition
    if (pThis->freeze && !pThis->wasFrozen) {
        // Rising edge: Capture current write head as the start of our loop
        pThis->freezeStartPos = pThis->writeHead;
        pThis->loopPhase = 0;
    }
    pThis->wasFrozen = pThis->freeze;

    for (int i = 0; i < numFrames; ++i) {
        // 1. Read Input
        float inSampleL = dryL[i];
        float inSampleR = dryR[i];

        // 2. Read from Delay Buffer (with Windowing if Frozen)
        float delayL, delayR;

        if (pThis->freeze) {
            uint32_t L = delayLen;
            if (pThis->loopPhase >= L) pThis->loopPhase = 0; // Safety clamp if L changed

            // Calculate Window Size
            uint32_t W = (uint32_t)(L * (pThis->crossfade / 100.0f));
            if (W < 1) W = 1;

            // Pointer A (Main Loop)
            // We read backwards from the freeze point by L, then add phase
            uint32_t ptrA = (pThis->freezeStartPos + pThis->bufferSize - L + pThis->loopPhase) % pThis->bufferSize;
            delayL = pThis->audioBuffer[ptrA * 2];
            delayR = pThis->audioBuffer[ptrA * 2 + 1];

            // Crossfade Window (Dual Pointer)
            if (pThis->loopPhase >= (L - W)) {
                float t = (float)(pThis->loopPhase - (L - W)) / (float)W; // 0.0 to 1.0
                float gainA = getSineInterp(1.0f - t); // cos(x) = sin(pi/2 - x)
                float gainB = getSineInterp(t);        // sin(x)

                // Pointer B (Start of Loop)
                uint32_t offsetB = pThis->loopPhase - (L - W);
                uint32_t ptrB = (pThis->freezeStartPos + pThis->bufferSize - L + offsetB) % pThis->bufferSize;
                
                delayL = delayL * gainA + pThis->audioBuffer[ptrB * 2] * gainB;
                delayR = delayR * gainA + pThis->audioBuffer[ptrB * 2 + 1] * gainB;
            }
        } else {
            uint32_t readPos = (pThis->writeHead + pThis->bufferSize - delayLen) % pThis->bufferSize;
            delayL = pThis->audioBuffer[readPos * 2];
            delayR = pThis->audioBuffer[readPos * 2 + 1];
        }

        // 3. Process Effects on the Wet Signal (Buffer Read)
        // Bit Crush
        delayL = bitCrush(delayL, pThis->bitDepth);
        delayR = bitCrush(delayR, pThis->bitDepth);

        // Filters (Base/Width)
        delayL = processFilter(delayL, pThis->hpfStateL[0], hpfCoeff, true); // HPF
        delayL = processFilter(delayL, pThis->lpfStateL[0], lpfCoeff, false); // LPF
        delayR = processFilter(delayR, pThis->hpfStateR[0], hpfCoeff, true);
        delayR = processFilter(delayR, pThis->lpfStateR[0], lpfCoeff, false);

        // Tape Saturation
        if (pThis->tapeSat > 0.0f) {
            float drive = 1.0f + pThis->tapeSat * 4.0f;
            delayL = processTape(delayL, drive, pThis->tapeL, pThis->headBumpCoeffs, pThis->hfRollOffCoeffs);
            delayR = processTape(delayR, drive, pThis->tapeR, pThis->headBumpCoeffs, pThis->hfRollOffCoeffs);
        }

        // 4. Output Mixing
        // If Frozen: Dry is muted, only Wet (Buffer) is heard
        if (pThis->freeze) {
            outL[i] = delayL;
            outR[i] = delayR;
        } else {
            outL[i] = inSampleL + delayL;
            outR[i] = inSampleR + delayR;
        }

        // 5. Feedback & Write to Buffer
        float fbSourceL = delayL;
        float fbSourceR = delayR;

        if (fbInL) fbSourceL = fbInL[i];
        if (fbInR) fbSourceR = fbInR[i];

        // If Frozen: Input is cut, Feedback is effectively 100% (looping)
        float fbVal = pThis->freeze ? 1.0f : pThis->feedback;
        float writeL = (pThis->freeze ? 0.0f : inSampleL) + fbSourceL * fbVal;
        float writeR = (pThis->freeze ? 0.0f : inSampleR) + fbSourceR * fbVal;

        // Ping Pong Swap
        if (pThis->pingPong) {
            float temp = writeL;
            writeL = writeR;
            writeR = temp;
        }

        // Auto-Limiter (Safety)
        float maxAbs = my_max(my_abs(writeL), my_abs(writeR));
        if (maxAbs > pThis->limiterEnvelope) {
            pThis->limiterEnvelope = maxAbs;
        } else {
            pThis->limiterEnvelope *= releaseFactor;
        }
        if (pThis->limiterEnvelope > threshold) {
            float gain = threshold / pThis->limiterEnvelope;
            writeL *= gain;
            writeR *= gain;
        }

        if (pThis->freeze) {
            // Destructive Loop Overdub
            uint32_t L = delayLen;
            uint32_t writePos = (pThis->freezeStartPos + pThis->bufferSize - L + pThis->loopPhase) % pThis->bufferSize;
            pThis->audioBuffer[writePos * 2] = writeL;
            pThis->audioBuffer[writePos * 2 + 1] = writeR;
            pThis->loopPhase++;
        } else {
            pThis->audioBuffer[pThis->writeHead * 2] = writeL;
            pThis->audioBuffer[pThis->writeHead * 2 + 1] = writeR;
            pThis->writeHead = (pThis->writeHead + 1) % pThis->bufferSize;
        }
    }

    // Update MIDI clock counter
    pThis->framesSinceLastMidiClock += numFrames;
}

bool draw(_NT_algorithm* self) {
    MrFreezeAlgorithm* pThis = (MrFreezeAlgorithm*)self;
    NT_drawText(10, 10, pThis->freeze ? "FROZEN" : "MrFreeze");
    
    char buf[32];
    buf[0] = 'F'; buf[1] = 'B'; buf[2] = ':'; buf[3] = ' ';
    int fb = (int)(pThis->feedback * 100.0f);
    int len = NT_intToString(buf + 4, fb);
    buf[4 + len] = '%';
    buf[4 + len + 1] = 0;
    NT_drawText(10, 25, buf);

    // Draw Waveform Visualization (Bottom half: y=32 to 63)
    // Clear background
    NT_drawShapeI(kNT_rectangle, 0, 32, 255, 63, 0);
    // Draw border
    NT_drawShapeI(kNT_box, 0, 32, 255, 63, 5);

    uint32_t L = pThis->currentDelayLen;
    if (L < 1) L = 1;
    
    // Determine end position (most recent write or freeze point)
    uint32_t endPos = pThis->freeze ? pThis->freezeStartPos : pThis->writeHead;
    
    int yBase = 48;
    int lastY = yBase;
    
    // Draw waveform
    for (int x = 0; x < 256; ++x) {
        // Map pixel x to buffer index
        // We visualize the loop/delay window [endPos - L, endPos]
        uint32_t offset = (uint32_t)((uint64_t)x * L / 256);
        uint32_t idx = (endPos + pThis->bufferSize - L + offset) % pThis->bufferSize;
        
        float sample = pThis->audioBuffer[idx * 2]; // Left channel only for simplicity
        int y = yBase - (int)(sample * 15.0f); // Scale +/- 15 pixels
        if (y < 33) y = 33;
        if (y > 62) y = 62;
        
        if (x > 0) {
            NT_drawShapeI(kNT_line, x - 1, lastY, x, y, 15);
        }
        lastY = y;
    }

    if (pThis->freeze) {
        // Draw Play Head
        int xPhase = (int)((uint64_t)pThis->loopPhase * 256 / L);
        if (xPhase >= 0 && xPhase < 256) {
            NT_drawShapeI(kNT_line, xPhase, 32, xPhase, 63, 10);
        }
        // Draw Crossfade Point (End of loop)
        int wPix = (int)(256.0f * pThis->crossfade / 100.0f);
        int xFade = 256 - wPix;
        if (xFade < 256 && xFade >= 0) {
             NT_drawShapeI(kNT_line, xFade, 32, xFade, 63, 8);
        }
    }

    return false;
}

// Factory definition
static const _NT_factory factory = {
    .guid = kGuid,
    .name = "MrFreeze",
    .description = "Rhythmic performance delay and looper.",
    .numSpecifications = kNumSpecifications,
    .specifications = specificationsInfo,
    .calculateRequirements = calculateRequirements,
    .construct = construct,
    .parameterChanged = parameterChanged,
    .step = step,
    .draw = draw,
    .midiRealtime = midiRealtime,
    .tags = (uint32_t)(kNT_tagEffect | kNT_tagDelay),
};

// Plugin entry point
uintptr_t pluginEntry(_NT_selector selector, uint32_t data) {
    switch (selector) {
        case kNT_selector_version:
            return kNT_apiVersionCurrent;
        case kNT_selector_numFactories:
            return 1;
        case kNT_selector_factoryInfo:
            return (uintptr_t)((data == 0) ? &factory : nullptr);
    }
    return 0;
}

} // extern "C"