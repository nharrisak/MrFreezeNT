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
    int midiChannel;
    float tempo;
    float feedback;
    float resonance;
    float base, width;
    int bitDepth;
    float tapeSat;

    // State
    uint32_t bufferSize;
    uint32_t writeHead;
    BiquadState lpfStateL, lpfStateR;
    BiquadState hpfStateL, hpfStateR;

    // Clock State
    float lastClockSample;
    uint32_t framesSinceLastClock;
    float detectedBpm;

    // MIDI Clock State
    uint32_t framesSinceLastMidiClock;
    float detectedMidiBpm;
    int activeNoteCount;

    // Limiter State
    float limiterEnvelope;

    // Loop State
    uint32_t loopPhase;
    uint32_t freezeStartPos;
    bool wasFrozen;
    uint32_t currentDelayLen;
    float currentDryGain;

    // Tape State
    TapeState tapeL, tapeR;
    float lastSampleRate;
    float lastTapeSat;
    float smoothedFBase;
    float smoothedFWidth;
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
    kParam_MidiChannel,
    kParam_Tempo,
    kParam_Feedback,
    kParam_Resonance,
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
    [kParam_MidiChannel] = { .name = "MIDI Ch", .min = 0, .max = 16, .def = 0, .unit = kNT_unitNone },
    [kParam_Tempo]    = { .name = "Tempo",     .min = 30, .max = 300, .def = 120, .unit = kNT_unitBPM },

    // Page 3: Character & Filter
    [kParam_Feedback] = { .name = "Feedback", .min = 0, .max = 100, .def = 50, .unit = kNT_unitPercent },
    [kParam_Resonance]= { .name = "Resonance",.min = 0, .max = 100, .def = 30, .unit = kNT_unitPercent },
    [kParam_Base]     = { .name = "Base",     .min = 0, .max = 127, .def = 0, .unit = kNT_unitNone },
    [kParam_Width]    = { .name = "Width",    .min = 0, .max = 127, .def = 127, .unit = kNT_unitNone },
    [kParam_BitDepth] = { .name = "Bit Depth",.min = 1, .max = 24, .def = 24, .unit = kNT_unitNone },
    [kParam_LoFiSR]   = { .name = "Lo-Fi SR", .min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent },
    [kParam_TapeSat]  = { .name = "Tape Sat", .min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent },
};

// Parameter pages
static const uint8_t page1[] = { kParam_DryInL, kParam_DryInR, kParam_FBInL, kParam_FBInR, kParam_ClockIn, kParam_OutL, kParam_OutR };
static const uint8_t page2[] = { kParam_Freeze, kParam_Division, kParam_Triplet, kParam_Dotted, kParam_Crossfade, kParam_PingPong, kParam_Sync, kParam_MidiChannel, kParam_Tempo };
static const uint8_t page3[] = { kParam_Feedback, kParam_Resonance, kParam_Base, kParam_Width, kParam_BitDepth, kParam_LoFiSR, kParam_TapeSat };

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

// Helper: Calculate Biquad Coeffs (RBJ)
enum { kFilterLPF, kFilterHPF };

static void calcFilterCoeffs(int type, float fNorm, float Q, float* c) {
    // fNorm = f / Sr. Max 0.5.
    // omega = 2 * pi * fNorm.
    // phase (0..1 for 0..pi/2) = 4 * fNorm.
    
    float phase = 4.0f * fNorm;
    float sn, cs;
    
    if (phase <= 1.0f) {
        sn = getSineInterp(phase);
        cs = getSineInterp(1.0f - phase);
    } else {
        // phase 1..2 (pi/2 to pi)
        sn = getSineInterp(2.0f - phase);
        cs = -getSineInterp(phase - 1.0f);
    }
    
    float alpha = sn / (2.0f * Q);
    float a0_inv = 1.0f / (1.0f + alpha);
    
    if (type == kFilterLPF) {
        float b1 = 1.0f - cs;
        float b0 = b1 * 0.5f;
        c[0] = b0 * a0_inv;
        c[1] = b1 * a0_inv;
        c[2] = b0 * a0_inv;
        c[3] = (-2.0f * cs) * a0_inv;
        c[4] = (1.0f - alpha) * a0_inv;
    } else { // HPF
        float b1 = -(1.0f + cs);
        float b0 = -b1 * 0.5f;
        c[0] = b0 * a0_inv;
        c[1] = b1 * a0_inv;
        c[2] = b0 * a0_inv;
        c[3] = (-2.0f * cs) * a0_inv;
        c[4] = (1.0f - alpha) * a0_inv;
    }
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
    alg->resonance = 0.3f;
    alg->freeze = false;
    alg->bitDepth = 24;
    alg->base = 0.0f;
    alg->width = 1.0f;
    alg->dryInL = 1;
    alg->dryInR = 2;
    alg->outL = 13;
    alg->outR = 14;
    alg->sync = 1;
    alg->midiChannel = 0;
    alg->division = 4;
    alg->tempo = 120.0f;
    alg->lastClockSample = 0.0f;
    alg->framesSinceLastClock = 0;
    alg->detectedBpm = 120.0f;
    alg->framesSinceLastMidiClock = 0;
    alg->detectedMidiBpm = 120.0f;
    alg->activeNoteCount = 0;
    alg->limiterEnvelope = 0.0f;
    alg->loopPhase = 0;
    alg->freezeStartPos = 0;
    alg->wasFrozen = false;
    alg->currentDelayLen = 0;
    alg->currentDryGain = 1.0f;
    alg->crossfade = 10.0f;
    alg->lpfStateL = {};
    alg->lpfStateR = {};
    alg->hpfStateL = {};
    alg->hpfStateR = {};
    
    // Init Tape State
    alg->tapeL = {};
    alg->tapeR = {};
    alg->lastSampleRate = 0.0f;
    alg->lastTapeSat = -1.0f;
    alg->smoothedFBase = 0.0005f;
    alg->smoothedFWidth = 0.4505f;
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
        case kParam_Crossfade: pThis->crossfade = (float)self->v[p]; break;
        case kParam_Feedback: pThis->feedback = self->v[p] / 100.0f; break;
        case kParam_Resonance: pThis->resonance = self->v[p] / 100.0f; break;
        case kParam_PingPong: pThis->pingPong = (self->v[p] > 0.5f); break;
        case kParam_Sync:   pThis->sync = (int)self->v[p]; break;
        case kParam_MidiChannel: pThis->midiChannel = (int)self->v[p]; break;
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

void midiMessage(_NT_algorithm* self, uint8_t byte0, uint8_t byte1, uint8_t byte2) {
    MrFreezeAlgorithm* pThis = (MrFreezeAlgorithm*)self;
    int status = byte0 & 0xF0;
    int channel = byte0 & 0x0F;

    if (pThis->midiChannel > 0 && channel != (pThis->midiChannel - 1)) {
        return;
    }
    
    if (status == 0x90 && byte2 > 0) { // Note On
        pThis->activeNoteCount++;
        
        // Map note to division (wrap around octave)
        int div = byte1 % 12;
        if (div <= 6) {
            NT_setParameterFromAudio(NT_algorithmIndex(self), kParam_Division + NT_parameterOffset(), div);
        }
        
        if (pThis->activeNoteCount == 1) {
            NT_setParameterFromAudio(NT_algorithmIndex(self), kParam_Freeze + NT_parameterOffset(), 1);
        }
    } else if (status == 0x80 || (status == 0x90 && byte2 == 0)) { // Note Off
        pThis->activeNoteCount--;
        if (pThis->activeNoteCount <= 0) {
            pThis->activeNoteCount = 0;
            NT_setParameterFromAudio(NT_algorithmIndex(self), kParam_Freeze + NT_parameterOffset(), 0);
        }
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

    // Filter Coeffs (2-pole Resonant)
    float hpfCoeffs[5], lpfCoeffs[5];
    float Q = 0.707f + pThis->resonance * 9.0f; // Q from 0.7 to ~10
    
    // Map Base (0-1) to Freq Norm (20Hz - 20kHz approx)
    // fNorm = 0.0005 + val^3 * 0.45
    float baseSq = pThis->base * pThis->base * pThis->base;
    float fBase = 0.0005f + baseSq * 0.45f;
    float widthVal = my_min(1.0f, pThis->base + pThis->width);
    float widthSq = widthVal * widthVal * widthVal;
    float fWidth = 0.0005f + widthSq * 0.45f;

    // Smoothing
    float smoothing = 0.001f * numFrames;
    if (smoothing > 1.0f) smoothing = 1.0f;
    pThis->smoothedFBase += (fBase - pThis->smoothedFBase) * smoothing;
    pThis->smoothedFWidth += (fWidth - pThis->smoothedFWidth) * smoothing;

    calcFilterCoeffs(kFilterHPF, pThis->smoothedFBase, Q, hpfCoeffs);
    calcFilterCoeffs(kFilterLPF, pThis->smoothedFWidth, Q, lpfCoeffs);

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

    // Calculate Window Size
    uint32_t W = (uint32_t)(delayLen * (pThis->crossfade / 100.0f));
    if (W < 1) W = 1;
    float dryFadeStep = 1.0f / (float)W;

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

        // Update Dry Gain
        if (pThis->freeze) {
            pThis->currentDryGain -= dryFadeStep;
            if (pThis->currentDryGain < 0.0f) pThis->currentDryGain = 0.0f;
        } else {
            pThis->currentDryGain += dryFadeStep;
            if (pThis->currentDryGain > 1.0f) pThis->currentDryGain = 1.0f;
        }

        // 2. Read from Delay Buffer (with Windowing if Frozen)
        float delayL, delayR;

        if (pThis->freeze) {
            uint32_t L = delayLen;
            if (pThis->loopPhase >= L) pThis->loopPhase = 0; // Safety clamp if L changed

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
        delayL = processBiquad(delayL, pThis->hpfStateL, hpfCoeffs); // HPF
        delayL = processBiquad(delayL, pThis->lpfStateL, lpfCoeffs); // LPF
        delayR = processBiquad(delayR, pThis->hpfStateR, hpfCoeffs);
        delayR = processBiquad(delayR, pThis->lpfStateR, lpfCoeffs);

        // Tape Saturation
        if (pThis->tapeSat > 0.0f) {
            float drive = 1.0f + pThis->tapeSat * 4.0f;
            delayL = processTape(delayL, drive, pThis->tapeL, pThis->headBumpCoeffs, pThis->hfRollOffCoeffs);
            delayR = processTape(delayR, drive, pThis->tapeR, pThis->headBumpCoeffs, pThis->hfRollOffCoeffs);
        }

        // 4. Output Mixing
        // Apply inverse ramp to Wet signal to prevent pops
        float gainWet = getSineInterp(1.0f - pThis->currentDryGain);
        outL[i] = (inSampleL * pThis->currentDryGain) + (delayL * gainWet);
        outR[i] = (inSampleR * pThis->currentDryGain) + (delayR * gainWet);

        // 5. Feedback & Write to Buffer
        float fbSourceL = delayL;
        float fbSourceR = delayR;

        if (fbInL) fbSourceL = fbInL[i];
        if (fbInR) fbSourceR = fbInR[i];

        float t = 1.0f - pThis->currentDryGain;
        float fbVal = pThis->feedback + (1.0f - pThis->feedback) * t;
        float writeL = (inSampleL * pThis->currentDryGain) + fbSourceL * fbVal;
        float writeR = (inSampleR * pThis->currentDryGain) + fbSourceR * fbVal;

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
    .midiMessage = midiMessage,
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