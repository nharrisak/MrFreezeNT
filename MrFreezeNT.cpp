#include <distingnt/api.h>
#include <new>
#include <cstddef>
#include <cstring>
#include <math.h>

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
    bool outLMode, outRMode;
    bool freeze;
    int division;
    int triplet;
    bool dotted;
    float crossfade;
    bool pingPong;
    int loopMode;
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
    float limiterGain;

    // Loop State
    bool wasFrozen;
    uint32_t currentDelayLen;
    float currentDryGain;

    // Tape State
    TapeState tapeL, tapeR;
    float lastSampleRate;
    float lastTapeSat;
    float smoothedFBase;
    float smoothedFWidth;
    float smoothedDelayLen;
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
    kParam_OutL_Mode,
    kParam_OutR,
    kParam_OutR_Mode,
    kParam_Freeze,
    kParam_Division,
    kParam_Triplet,
    kParam_Dotted,
    kParam_Crossfade,
    kParam_PingPong,
    kParam_LoopMode,
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
static const char* const s_divs[] = { "1/64", "1/32", "1/16", "1/8", "1/4", "1/2", "1/1", "2/1", "4/1", nullptr };
static const char* const s_triplet[] = { "Off", "x3", "/3", nullptr };
static const char* const s_dotted[] = { "Off", "x1.5", nullptr };
static const char* const s_loopMode[] = { "Fwd", "Rev", "PingPong", nullptr };
static const char* const s_sync[] = { "Int", "Clock", "MIDI", nullptr };

// Parameter definitions
static const _NT_parameter parameters[] = {
    // Page 1: Routing & I/O
    NT_PARAMETER_AUDIO_INPUT("Dry In L", 0, 1)
    NT_PARAMETER_AUDIO_INPUT("Dry In R", 0, 2)
    NT_PARAMETER_AUDIO_INPUT("FB In L", 0, 0)
    NT_PARAMETER_AUDIO_INPUT("FB In R", 0, 0)
    NT_PARAMETER_CV_INPUT("Clock In", 0, 0)
    NT_PARAMETER_AUDIO_OUTPUT_WITH_MODE("Out L", 1, 13)
    NT_PARAMETER_AUDIO_OUTPUT_WITH_MODE("Out R", 1, 14)

    // Page 2: Performance & Timing
    [kParam_Freeze]   = { .name = "Freeze",    .min = 0, .max = 1, .def = 0, .unit = kNT_unitEnum, .enumStrings = s_offOn },
    [kParam_Division] = { .name = "Division",  .min = 0, .max = 8, .def = 4, .unit = kNT_unitEnum, .enumStrings = s_divs },
    [kParam_Triplet]  = { .name = "Triplet",   .min = 0, .max = 2, .def = 0, .unit = kNT_unitEnum, .enumStrings = s_triplet },
    [kParam_Dotted]   = { .name = "Dotted",    .min = 0, .max = 1, .def = 0, .unit = kNT_unitEnum, .enumStrings = s_dotted },
    [kParam_Crossfade]= { .name = "Crossfade %",.min = 0, .max = 50, .def = 10, .unit = kNT_unitPercent },
    [kParam_PingPong] = { .name = "Ping Pong", .min = 0, .max = 1, .def = 0, .unit = kNT_unitEnum, .enumStrings = s_offOn },
    [kParam_LoopMode] = { .name = "Loop Mode", .min = 0, .max = 2, .def = 0, .unit = kNT_unitEnum, .enumStrings = s_loopMode },
    [kParam_Sync]     = { .name = "Sync",      .min = 0, .max = 2, .def = 1, .unit = kNT_unitEnum, .enumStrings = s_sync },
    [kParam_MidiChannel] = { .name = "MIDI Ch", .min = 0, .max = 16, .def = 0, .unit = kNT_unitNone },
    [kParam_Tempo]    = { .name = "Tempo",     .min = 30, .max = 300, .def = 120, .unit = kNT_unitBPM },

    // Page 3: Character & Filter
    [kParam_Feedback] = { .name = "Feedback", .min = 0, .max = 100, .def = 100, .unit = kNT_unitPercent },
    [kParam_Resonance]= { .name = "Resonance",.min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent },
    [kParam_Base]     = { .name = "Base",     .min = 0, .max = 127, .def = 0, .unit = kNT_unitNone },
    [kParam_Width]    = { .name = "Width",    .min = 0, .max = 127, .def = 127, .unit = kNT_unitNone },
    [kParam_BitDepth] = { .name = "Bit Depth",.min = 1, .max = 24, .def = 24, .unit = kNT_unitNone },
    [kParam_LoFiSR]   = { .name = "Lo-Fi SR", .min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent },
    [kParam_TapeSat]  = { .name = "Tape Sat", .min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent },
};

// Parameter pages
static const uint8_t page1[] = { kParam_DryInL, kParam_DryInR, kParam_FBInL, kParam_FBInR, kParam_ClockIn, kParam_OutL, kParam_OutL_Mode, kParam_OutR, kParam_OutR_Mode };
static const uint8_t page2[] = { kParam_Freeze, kParam_Division, kParam_Triplet, kParam_Dotted, kParam_Crossfade, kParam_PingPong, kParam_LoopMode, kParam_Sync, kParam_MidiChannel, kParam_Tempo };
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
    if (idx > 31) idx = 31;
    float frac = idxF - (float)idx;
    return kSineTable[idx] * (1.0f - frac) + kSineTable[idx + 1] * frac;
}

// Table for 20^x, x=[0,1]
static const float kPow20Table[33] = {
    1.000000f, 1.098023f, 1.205651f, 1.323830f, 1.453593f, 1.596073f, 1.752519f, 1.924298f,
    2.112912f, 2.320013f, 2.547414f, 2.797105f, 3.071271f, 3.372312f, 3.702863f, 4.065812f,
    4.464336f, 4.901922f, 5.382401f, 5.909979f, 6.489268f, 7.125339f, 7.823759f, 8.590639f,
    9.432686f, 10.357273f, 11.372488f, 12.487214f, 13.711209f, 15.055179f, 16.530886f, 18.151241f,
    20.000000f
};

// Table for 1000^x, x=[0,1]
static const float kPow1000Table[33] = {
    1.000000f, 1.240937f, 1.539927f, 1.910953f, 2.371374f, 2.942727f, 3.651741f, 4.531584f,
    5.623413f, 6.978306f, 8.659643f, 10.746078f, 13.335214f, 16.548171f, 20.535250f, 25.482969f,
    31.622777f, 39.241898f, 48.696753f, 60.429639f, 74.989421f, 93.057204f, 115.478198f, 143.301257f,
    177.827941f, 220.673407f, 273.841963f, 339.820833f, 421.696503f, 523.299115f, 649.381632f, 805.842188f,
    1000.000000f
};

static inline float getLinearInterp(float phase01, const float* table) {
    if (phase01 <= 0.0f) return table[0];
    if (phase01 >= 1.0f) return table[32];
    float idxF = phase01 * 32.0f;
    int idx = (int)idxF;
    if (idx > 31) idx = 31;
    float frac = idxF - (float)idx;
    return table[idx] * (1.0f - frac) + table[idx + 1] * frac;
}

// Helper: Biquad Process
static inline float processBiquad(float x, BiquadState& s, const float* c) {
    // y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
    float y = c[0]*x + c[1]*s.x1 + c[2]*s.x2 - c[3]*s.y1 - c[4]*s.y2;
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
    std::memset(alg->audioBuffer, 0, req.dram);
    
    // Initialize state
    int maxTime = specifications ? specifications[kSpec_MaxDelayTime] : 10;
    alg->bufferSize = maxTime * 96000; // Stereo frames
    alg->writeHead = 0;
    
    // Initialize defaults
    alg->feedback = 1.0f;
    alg->resonance = 0.0f;
    alg->freeze = false;
    alg->loopMode = 0;
    alg->bitDepth = 24;
    alg->base = 0.0f;
    alg->width = 1.0f;
    alg->dryInL = 0;
    alg->dryInR = 1;
    alg->outL = 13;
    alg->outR = 14;
    alg->outLMode = 0;
    alg->outRMode = 0;
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
    alg->limiterGain = 1.0f;
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
    alg->smoothedDelayLen = 48000.0f;
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
        case kParam_OutL_Mode: pThis->outLMode = (bool)self->v[p]; break;
        case kParam_OutR:   pThis->outR = (int)self->v[p]; break;
        case kParam_OutR_Mode: pThis->outRMode = (bool)self->v[p]; break;
        case kParam_Freeze: pThis->freeze = (self->v[p] > 0.5f); break;
        case kParam_Division: pThis->division = (int)self->v[p]; break;
        case kParam_Triplet: pThis->triplet = (int)self->v[p]; break;
        case kParam_Dotted: pThis->dotted = (self->v[p] > 0.5f); break;
        case kParam_Crossfade: pThis->crossfade = (float)self->v[p]; break;
        case kParam_Feedback: pThis->feedback = self->v[p] / 100.0f; break;
        case kParam_Resonance: pThis->resonance = self->v[p] / 100.0f; break;
        case kParam_PingPong: pThis->pingPong = (self->v[p] > 0.5f); break;
        case kParam_LoopMode: pThis->loopMode = (int)self->v[p]; break;
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
        if (div <= 8) {
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
    if (numFrames <= 0) return;

    // Pointers to IO
    int dryInLIdx = (pThis->dryInL < 1) ? 1 : pThis->dryInL;
    const float* dryL = busFrames + (dryInLIdx - 1) * numFrames;
    
    int dryInRIdx = pThis->dryInR;
    const float* dryR = (dryInRIdx > 0) ? busFrames + (dryInRIdx - 1) * numFrames : dryL;
    
    int outLIdx = (pThis->outL < 1) ? 1 : pThis->outL;
    float* outL = busFrames + (outLIdx - 1) * numFrames;
    bool replaceL = pThis->outLMode;
    
    int outRIdx = (pThis->outR < 1) ? 1 : pThis->outR;
    float* outR = busFrames + (outRIdx - 1) * numFrames;
    bool replaceR = pThis->outRMode;

    // Clock Input Pointer
    int clockInIdx = pThis->clockIn;
    const float* clk = (clockInIdx > 0) ? busFrames + (clockInIdx - 1) * numFrames : nullptr;

    // Feedback Input Pointers
    int fbInLIdx = pThis->fbInL;
    int fbInRIdx = pThis->fbInR;
    const float* fbInL = (fbInLIdx > 0) ? busFrames + (fbInLIdx - 1) * numFrames : nullptr;
    const float* fbInR = (fbInRIdx > 0) ? busFrames + (fbInRIdx - 1) * numFrames : nullptr;

    // Filter Coeffs (2-pole Resonant)
    float hpfCoeffs[5], lpfCoeffs[5];
    float Q = 0.707f * getLinearInterp(pThis->resonance, kPow20Table); // Logarithmic Q from 0.7 to ~14
    
    // Map Base (0-1) to Freq Norm (20Hz - 20kHz approx)
    // Logarithmic mapping: f = f_min * (ratio)^val
    float fBase = 0.00025f * getLinearInterp(pThis->base, kPow1000Table);
    float widthVal = my_min(1.0f, pThis->base + pThis->width);
    float fWidth = 0.00025f * getLinearInterp(widthVal, kPow1000Table);

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

    // Limiter constants (Sample-rate independent)
    float lookAheadMs = 2.0f;
    uint32_t lookAheadSamples = (uint32_t)(sr * (lookAheadMs * 0.001f));
    float atkCoeff = 1.0f - expf(-1.0f / (sr * 0.001f)); // 1ms Attack
    float relCoeff = 1.0f - expf(-1.0f / (sr * 0.050f)); // 50ms Release
    const float kLimThreshold = 0.944f; // -0.5 dB

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
        case 7: beats = 8.0f; break;
        case 8: beats = 16.0f; break;
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

    // Calculate Window Size for Dry/Wet Fade
    uint32_t W_fade = (uint32_t)(delayLen * (pThis->crossfade / 100.0f));
    if (W_fade < 1) W_fade = 1;
    float dryFadeStep = 1.0f / (float)W_fade;

    // Handle Freeze Transition
    if (pThis->freeze && !pThis->wasFrozen) {
        // No special state reset needed for continuous tape model
    }
    pThis->wasFrozen = pThis->freeze;

    bool bypassHPF = (pThis->base <= 0.0f);
    bool bypassLPF = ((pThis->base + pThis->width) >= 1.0f);

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

        // 1. Smooth the delay length to prevent clicks and allow "tape" pitch effects
        float targetLen = (float)pThis->currentDelayLen;
        pThis->smoothedDelayLen += (targetLen - pThis->smoothedDelayLen) * 0.001f;

        // 2. Calculate Fractional Read Position
        // We look "backwards" from the current write head
        float readPos = (float)pThis->writeHead - pThis->smoothedDelayLen;

        // Wrap the floating point pointer within the circular buffer
        while (readPos < 0.0f) readPos += (float)pThis->bufferSize;
        while (readPos >= (float)pThis->bufferSize) readPos -= (float)pThis->bufferSize;

        // 3. Linear Interpolation
        uint32_t idxA = (uint32_t)readPos;
        uint32_t idxB = (idxA + 1) % pThis->bufferSize;
        float fraction = readPos - (float)idxA;

        // Sample the two nearest points and blend
        float delayL = pThis->audioBuffer[idxA * 2] * (1.0f - fraction) + 
                       pThis->audioBuffer[idxB * 2] * fraction;
        float delayR = pThis->audioBuffer[idxA * 2 + 1] * (1.0f - fraction) + 
                       pThis->audioBuffer[idxB * 2 + 1] * fraction;

        // 3. Process Effects on the Wet Signal (Buffer Read)
        // Bit Crush
        if (pThis->bitDepth < 24) {
            delayL = bitCrush(delayL, pThis->bitDepth);
            delayR = bitCrush(delayR, pThis->bitDepth);
        }

        // Filters (Base/Width)
        if (!bypassHPF) delayL = processBiquad(delayL, pThis->hpfStateL, hpfCoeffs); // HPF
        if (!bypassLPF) delayL = processBiquad(delayL, pThis->lpfStateL, lpfCoeffs); // LPF
        if (!bypassHPF) delayR = processBiquad(delayR, pThis->hpfStateR, hpfCoeffs);
        if (!bypassLPF) delayR = processBiquad(delayR, pThis->lpfStateR, lpfCoeffs);

        // Tape Saturation
        if (pThis->tapeSat > 0.0f) {
            float drive = 1.0f + pThis->tapeSat * 4.0f;
            delayL = processTape(delayL, drive, pThis->tapeL, pThis->headBumpCoeffs, pThis->hfRollOffCoeffs);
            delayR = processTape(delayR, drive, pThis->tapeR, pThis->headBumpCoeffs, pThis->hfRollOffCoeffs);
        }

        // --- Transparent Look-Ahead Limiter ---
        // 1. Look-Ahead Detection
        float futureReadPos = readPos + (float)lookAheadSamples;
        while (futureReadPos < 0.0f) futureReadPos += (float)pThis->bufferSize;
        while (futureReadPos >= (float)pThis->bufferSize) futureReadPos -= (float)pThis->bufferSize;

        uint32_t fIdxA = (uint32_t)futureReadPos;
        uint32_t fIdxB = (fIdxA + 1) % pThis->bufferSize;
        float fFrac = futureReadPos - (float)fIdxA;

        float fL = pThis->audioBuffer[fIdxA * 2] * (1.0f - fFrac) + pThis->audioBuffer[fIdxB * 2] * fFrac;
        float fR = pThis->audioBuffer[fIdxA * 2 + 1] * (1.0f - fFrac) + pThis->audioBuffer[fIdxB * 2 + 1] * fFrac;
        float futureMax = my_max(my_abs(fL), my_abs(fR));

        // 2. Calculate Target Gain
        float targetGain = 1.0f;
        if (futureMax > kLimThreshold) {
            targetGain = kLimThreshold / futureMax;
        }

        // 3. Apply Smoothed Gain
        float currentCoeff = (targetGain < pThis->limiterGain) ? atkCoeff : relCoeff;
        pThis->limiterGain += currentCoeff * (targetGain - pThis->limiterGain);

        delayL *= pThis->limiterGain;
        delayR *= pThis->limiterGain;

        // 4. Output Mixing
        float gainDry = getSineInterp(pThis->currentDryGain);
        float gainWet = getSineInterp(1.0f - pThis->currentDryGain);
        float mixL = (inSampleL * gainDry) + (delayL * gainWet);
        float mixR = (inSampleR * gainDry) + (delayR * gainWet);

        if (replaceL) outL[i] = mixL;
        else          outL[i] += mixL;

        if (replaceR) outR[i] = mixR;
        else          outR[i] += mixR;

        // 5. Feedback & Write to Buffer
        float fbSourceL = delayL;
        float fbSourceR = delayR;

        if (fbInL) fbSourceL = fbInL[i];
        if (fbInR) fbSourceR = fbInR[i];

        float fbVal = pThis->feedback * pThis->feedback; // Audio taper (squared)
        
        // Fade feedback to 0% when recording (gainWet goes to 0)
        float currentFb = fbVal * gainWet;
        
        float writeL = (inSampleL * gainDry) + (fbSourceL * currentFb);
        float writeR = (inSampleR * gainDry) + (fbSourceR * currentFb);

        // Ping Pong Swap
        if (pThis->pingPong) {
            float temp = writeL;
            writeL = writeR;
            writeR = temp;
        }

        // 5. Feedback & Write Ahead
        // The writeHead ALWAYS moves forward, even in freeze, to record the "history"
        pThis->audioBuffer[pThis->writeHead * 2] = writeL;
        pThis->audioBuffer[pThis->writeHead * 2 + 1] = writeR;
        
        // Standard increment - the history is preserved behind us
        pThis->writeHead = (pThis->writeHead + 1) % pThis->bufferSize;
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

    // Draw BPM
    float currentBpm = pThis->tempo;
    if (pThis->sync == 1) currentBpm = pThis->detectedBpm;
    else if (pThis->sync == 2) currentBpm = pThis->detectedMidiBpm;
    buf[0] = 'B'; buf[1] = 'P'; buf[2] = 'M'; buf[3] = ':'; buf[4] = ' ';
    NT_floatToString(buf + 5, currentBpm, 1);
    NT_drawText(80, 25, buf);

    // Draw Waveform Visualization (Bottom half: y=32 to 63)
    // Clear background
    NT_drawShapeI(kNT_rectangle, 0, 32, 255, 63, 0);
    // Draw border
    NT_drawShapeI(kNT_box, 0, 32, 255, 63, 5);

    uint32_t L = pThis->currentDelayLen;
    if (L < 1) L = 1;
    
    // Determine end position (most recent write or freeze point)
    uint32_t endPos = pThis->writeHead;
    
    int yBase = 48;
    int lastY = yBase;
    
    // Draw waveform
    for (int x = 0; x < 256; ++x) {
        // Map pixel x to buffer index
        // We visualize the loop/delay window [endPos - L, endPos]
        uint32_t offset = (x * L) >> 8;
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
            return (uintptr_t)((data == 0) ? &factory : 0);
    }
    return 0;
}