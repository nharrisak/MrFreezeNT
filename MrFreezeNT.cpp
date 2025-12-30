#include <distingnt/api.h>
#include <new>
#include <cstddef>
#include <cstring>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Algorithm state
struct BiquadState {
    float x1, x2, y1, y2;
};

struct TapeState {
    BiquadState dcBlock;
    float lastPreSat;
};

// Diffusion Constants (Primes approx 20ms and 7ms at 96kHz)
static const int kApdSize1 = 3559;
static const int kApdSize2 = 337;
static const int kTotalApdSize = (kApdSize1 + kApdSize2) * 2;

struct AllPassState {
    float* buffer;
    int size;
    int head;
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
    float tone;
    int bitDepth;
    float tapeSat;
    float lofiSR;
    float diffusion;
    float drift;

    // State
    uint32_t bufferSize;
    uint32_t writeHead;
    BiquadState lpfStateL, lpfStateR;
    BiquadState hpfStateL, hpfStateR;
    BiquadState tiltStateL, tiltStateR;

    // Clock State (External CV)
    float lastClockSample;
    uint32_t framesSinceLastClock;
    float detectedBpm;             // Instantaneous BPM for display
    float stableExtBpm;            // BPM averaged over multiple pulses
    uint32_t extClockPulseCount;   // Count pulses for averaging
    uint32_t extClockAccumFrames;  // Accumulated frames over pulses

    // MIDI Clock State
    uint32_t framesSinceLastMidiClock;
    float detectedMidiBpm;         // Instantaneous BPM for display
    float stableMidiBpm;           // BPM updated only after full beat (24 pulses)
    uint32_t midiClockPulseCount;  // Count pulses (0-23)
    uint32_t midiClockAccumFrames; // Accumulated frames over 24 pulses
    
    // Shared sync state
    float smoothedDelayTarget;     // Smoothed target delay for hysteresis
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
    float smoothedBase;
    float smoothedWidth;
    float smoothedTone;
    float smoothedDelayLen;
    float fadeTargetLen;
    float fadeOldLen;
    float fadePhase;
    uint32_t rngState;
    float reversePhase;
    int lastLoopMode;
    float lofiCounter;
    float lastLofiSampleL;
    float lastLofiSampleR;
    float prevLofiSampleL;
    float prevLofiSampleR;
    float lofiPreFilterL;
    float lofiPreFilterR;
    float lofiPreFilter2L;  // Second pole for steeper anti-aliasing
    float lofiPreFilter2R;
    
    // Drift State
    float driftNoiseL, driftNoiseR;
    int driftUpdateCounter;
    float currentDriftDelay;
    float currentDriftWidth;
    float currentDriftBase;
    float currentDriftTone;
    float currentDriftLofi;
    float currentDriftDiff;
    float currentDriftFeedback;
    float smoothedLofiSR;
    float smoothedDiffusion;
    float smoothedFeedback;
    float fadeReadPos;
    float fadeInc;
    float prevReadPos;
    float lastReadInc;

    // Cached Filter Coefficients (avoid recalculating every step)
    float cachedHpfCoeffs[5];
    float cachedLpfCoeffs[5];
    float cachedDcCoeffs[5];
    float cachedTiltCoeffs[5];
    float lastCalcBase;       // Last smoothedBase used for HPF calc
    float lastCalcWidth;      // Last smoothedWidth used for LPF calc  
    float lastCalcTone;       // Last smoothedTone used for tilt calc
    float lastCalcResonance;  // Last resonance used for filter calc
    bool coeffsInitialized;

    // Diffusion State
    AllPassState apd1L, apd2L, apd1R, apd2R;
    float apdLfoPhase;
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
    kParam_Tone,
    kParam_BitDepth,
    kParam_LoFiSR,
    kParam_TapeSat,
    kParam_Diffusion,
    kParam_Drift,
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
    [kParam_MidiChannel] = { .name = "MIDI Ch", .min = 0, .max = 15, .def = 0, .unit = kNT_unitNone },
    [kParam_Tempo]    = { .name = "Tempo",     .min = 30, .max = 300, .def = 120, .unit = kNT_unitBPM },

    // Page 3: Character & Filter
    [kParam_Feedback] = { .name = "Feedback", .min = -60, .max = 6, .def = 0, .unit = kNT_unitDb_minInf },
    [kParam_Resonance]= { .name = "Resonance",.min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent },
    [kParam_Base]     = { .name = "Base",     .min = 0, .max = 127, .def = 0, .unit = kNT_unitNone },
    [kParam_Width]    = { .name = "Width",    .min = 0, .max = 127, .def = 127, .unit = kNT_unitNone },
    [kParam_Tone]     = { .name = "Tone",     .min = 0, .max = 100, .def = 50, .unit = kNT_unitPercent },
    [kParam_BitDepth] = { .name = "Bit Depth",.min = 1, .max = 24, .def = 24, .unit = kNT_unitNone },
    [kParam_LoFiSR]   = { .name = "Lo-Fi", .min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent },
    [kParam_TapeSat]  = { .name = "Tape Sat", .min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent },
    [kParam_Diffusion]= { .name = "Diffusion",.min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent },
    [kParam_Drift]    = { .name = "Drift",    .min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent },
};

// Parameter pages
static const uint8_t page1[] = { kParam_DryInL, kParam_DryInR, kParam_FBInL, kParam_FBInR, kParam_ClockIn, kParam_OutL, kParam_OutL_Mode, kParam_OutR, kParam_OutR_Mode };
static const uint8_t page2[] = { kParam_Freeze, kParam_Division, kParam_Triplet, kParam_Dotted, kParam_Crossfade, kParam_PingPong, kParam_LoopMode, kParam_Sync, kParam_MidiChannel, kParam_Tempo };
static const uint8_t page3[] = { kParam_Feedback, kParam_Resonance, kParam_Base, kParam_Width, kParam_Tone, kParam_BitDepth, kParam_LoFiSR, kParam_TapeSat, kParam_Diffusion, kParam_Drift };

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

// Wrap buffer position into range [0, bufferSize)
// More efficient than fmodf for buffer wrapping and doesn't require math library
static inline float wrapBufferPos(float pos, uint32_t bufferSize) {
    float size = (float)bufferSize;
    // Handle negative values
    if (pos < 0.0f) {
        // Add size until positive (at most one addition needed for normal cases)
        while (pos < 0.0f) pos += size;
        return pos;
    }
    // Handle values >= bufferSize
    if (pos >= size) {
        // Subtract size until in range (at most one subtraction needed for normal cases)
        while (pos >= size) pos -= size;
    }
    return pos;
}

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

// Cosine interpolation table: (1 - cos(t * PI)) / 2 for t = 0 to 1
// Gives smooth S-curve interpolation (slow start, fast middle, slow end)
static const float kCosineInterpTable[33] = {
    0.000000f, 0.002410f, 0.009607f, 0.021530f, 0.038060f, 0.059039f, 0.084265f, 0.113495f,
    0.146447f, 0.182803f, 0.222215f, 0.264302f, 0.308658f, 0.354858f, 0.402455f, 0.450991f,
    0.500000f, 0.549009f, 0.597545f, 0.645142f, 0.691342f, 0.735698f, 0.777785f, 0.817197f,
    0.853553f, 0.886505f, 0.915735f, 0.940961f, 0.961940f, 0.978470f, 0.990393f, 0.997590f,
    1.000000f
};

// Cosine interpolation: S-curve from 0 to 1 (smoother than linear)
static inline float getCosineInterp(float phase01) {
    if (phase01 <= 0.0f) return 0.0f;
    if (phase01 >= 1.0f) return 1.0f;
    float idxF = phase01 * 32.0f;
    int idx = (int)idxF;
    if (idx > 31) idx = 31;
    float frac = idxF - (float)idx;
    return kCosineInterpTable[idx] * (1.0f - frac) + kCosineInterpTable[idx + 1] * frac;
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

static inline float saturate(float x) {
    // Fast approximation of tanh(x)
    // Very stable for feedback loops
    if (x >= 3.0f) return 1.0f;
    if (x <= -3.0f) return -1.0f;
    return x * (27.0f + x * x) / (27.0f + 9.0f * x * x);
}

static inline float applyHysteresis(float in, float& state, float amount) {
    // amount = 0 (transparent) to 1 (heavy smear)
    float target = saturate(in);
    // A simple one-pole lag mimics the 'lag' in magnetic alignment
    float coeff = 0.7f + (0.25f * (1.0f - amount)); 
    state = state + coeff * (target - state);
    return state;
}

// Helper: Tape Saturation Chain
static inline float processTape(float x, float drive, float amount, TapeState& s, const float* dcCoeffs) {
    // 1. DC Blocker (Essential to prevent bias from exploding the loop)
    float in = processBiquad(x, s.dcBlock, dcCoeffs);
    
    // 2. Add Asymmetrical Bias (Even harmonics/Warmth)
    // Small offset before saturation creates 'warm' even-order harmonics
    float biased = (in * drive) + (0.01f * amount);

    // 3. Hysteresis / Smear (Memory Effect)
    // s.lastPreSat acts as the 'magnetic memory'
    float saturated = applyHysteresis(biased, s.lastPreSat, amount);

    // 4. Subtract Bias to center signal before HF roll-off
    saturated -= (0.005f * amount);

    // 5. Unity-Gain Blend
    return (x * (1.0f - amount)) + (saturated * amount);
}

// Helper: All-Pass Diffuser
static inline float processAllPass(float x, AllPassState& ap, float mod, float g) {
    // Modulated delay length (approx full buffer size minus margin)
    float d = (float)ap.size - 25.0f + mod; 
    
    float r = (float)ap.head - d;
    while (r < 0.0f) r += (float)ap.size;
    
    int i = (int)r;
    int j = i + 1; if (j >= ap.size) j = 0;
    float f = r - (float)i;
    
    float delayed = ap.buffer[i] * (1.0f - f) + ap.buffer[j] * f;
    float w = x + g * delayed;
    float y = -g * w + delayed;
    
    ap.buffer[ap.head++] = w;
    if (ap.head >= ap.size) ap.head = 0;
    return y;
}

// Helper: Nested All-Pass Diffuser (True Nesting)
static inline float processNestedAllPass(float x, AllPassState& outer, AllPassState& inner, float modOuter, float modInner, float g) {
    // Modulated delay length for outer
    float d = (float)outer.size - 25.0f + modOuter; 
    
    float r = (float)outer.head - d;
    while (r < 0.0f) r += (float)outer.size;
    
    int i = (int)r;
    int j = i + 1; if (j >= outer.size) j = 0;
    float f = r - (float)i;
    
    float delayed = outer.buffer[i] * (1.0f - f) + outer.buffer[j] * f;
    float feedback = processAllPass(delayed, inner, modInner, g);
    
    float w = x + g * feedback;
    float y = -g * w + feedback;
    
    outer.buffer[outer.head++] = w;
    if (outer.head >= outer.size) outer.head = 0;
    return y;
}

// Helper: Fast Random (LCG)
static inline float getNoise(uint32_t& state) {
    state = state * 1664525 + 1013904223;
    return (float)state * (1.0f / 4294967296.0f);
}

// Helper: Bit Crusher
static inline float bitCrush(float x, int depth, uint32_t& rngState) {
    if (depth >= 24) return x;
    
    // Calculate levels based on depth
    float levels = powf(2.0f, (float)depth); 
    float invLevels = 1.0f / levels;

    // TPDF Dither to mask quantization distortion
    float dither = (getNoise(rngState) - getNoise(rngState)) * invLevels;
    
    // Mid-tread quantization: ensures 0.0 stays 0.0
    return roundf((x + dither) * levels) * invLevels;
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

static void calcTiltCoeffs(float tone, float sr, float* c) {
    // tone is 0.0 (dark) to 1.0 (bright)
    float gain = (tone - 0.5f) * 2.0f; // -1.0 to 1.0
    float omega = 2.0f * M_PI * 1000.0f / sr;
    float sn = sinf(omega);
    float cs = cosf(omega);
    
    // Simplified shelving logic
    float A = powf(10.0f, (gain * 6.0f) / 40.0f); 
    float beta = sqrtf(A) / 0.707f;

    // Standard High-Shelf Coeffs (acts as tilt when centered)
    float ap1 = A + 1.0f;
    float am1 = A - 1.0f;
    float bsn = beta * sn;
    float a0 = ap1 - am1 * cs + bsn;
    float a0_inv = 1.0f / a0;

    c[0] = (A * (ap1 + am1 * cs + bsn)) * a0_inv;
    c[1] = (-2.0f * A * (am1 + ap1 * cs)) * a0_inv;
    c[2] = (A * (ap1 + am1 * cs - bsn)) * a0_inv;
    c[3] = (2.0f * (am1 - ap1 * cs)) * a0_inv;
    c[4] = (ap1 - am1 * cs - bsn) * a0_inv;
}

// --- Core Functions ---

void calculateRequirements(_NT_algorithmRequirements& req, const int32_t* specifications) {
    req.numParameters = kNumParameters;
    req.sram = sizeof(MrFreezeAlgorithm);
    
    // Calculate DRAM based on Max Delay Time (spec 0)
    // maxTime is in seconds, convert to samples: maxTime * sampleRate
    // Stereo (2 channels), float (4 bytes)
    int maxTime = specifications ? specifications[kSpec_MaxDelayTime] : 10;
    uint32_t sampleRate = NT_globals.sampleRate;
    req.dram = (maxTime * sampleRate * 2 + kTotalApdSize) * sizeof(float);
    
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
    uint32_t sampleRate = NT_globals.sampleRate;
    alg->bufferSize = maxTime * sampleRate; // Stereo frames (number of stereo sample pairs)
    alg->writeHead = 0;
    
    // Initialize defaults
    alg->feedback = 1.0f; // 0dB
    alg->resonance = 0.0f;
    alg->freeze = false;
    alg->loopMode = 0;
    alg->bitDepth = 24;
    alg->base = 0.0f;
    alg->width = 1.0f;
    alg->tone = 0.5f;
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
    alg->stableExtBpm = 120.0f;
    alg->extClockPulseCount = 0;
    alg->extClockAccumFrames = 0;
    alg->framesSinceLastMidiClock = 0;
    alg->detectedMidiBpm = 120.0f;
    alg->stableMidiBpm = 120.0f;
    alg->midiClockPulseCount = 0;
    alg->midiClockAccumFrames = 0;
    alg->smoothedDelayTarget = 0.0f;
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
    alg->tiltStateL = {};
    alg->tiltStateR = {};
    
    // Init Tape State
    alg->tapeL = {};
    alg->tapeR = {};
    alg->lastSampleRate = 0.0f;
    alg->lastTapeSat = -1.0f;
    alg->smoothedBase = 0.0f;
    alg->smoothedWidth = 1.0f;
    alg->smoothedTone = 0.5f;
    // Initialize delay lengths to 0.5 seconds at current sample rate
    float initialDelayLen = sampleRate * 0.5f;
    alg->smoothedDelayLen = initialDelayLen;
    alg->fadeTargetLen = initialDelayLen;
    alg->fadeOldLen = initialDelayLen;
    alg->fadePhase = 1.0f;
    // Coeffs will be calc'd in step
    alg->rngState = 0x5EED;
    alg->reversePhase = 0.0f;
    alg->lastLoopMode = 0;
    alg->lofiCounter = 0.0f;
    alg->lastLofiSampleL = 0.0f;
    alg->lastLofiSampleR = 0.0f;
    alg->prevLofiSampleL = 0.0f;
    alg->prevLofiSampleR = 0.0f;
    alg->lofiPreFilterL = 0.0f;
    alg->lofiPreFilterR = 0.0f;
    alg->lofiPreFilter2L = 0.0f;
    alg->lofiPreFilter2R = 0.0f;
    alg->lofiSR = 0.0f;
    alg->diffusion = 0.0f;
    alg->drift = 0.0f;
    alg->driftNoiseL = 0.0f;
    alg->driftNoiseR = 0.0f;
    alg->driftUpdateCounter = 0;
    alg->currentDriftDelay = 0.0f;
    alg->currentDriftWidth = 0.0f;
    alg->currentDriftBase = 0.0f;
    alg->currentDriftTone = 0.0f;
    alg->currentDriftLofi = 0.0f;
    alg->currentDriftDiff = 0.0f;
    alg->currentDriftFeedback = 0.0f;
    alg->smoothedLofiSR = 0.0f;
    alg->smoothedDiffusion = 0.0f;
    alg->smoothedFeedback = 1.0f;
    alg->fadeReadPos = 0.0f;
    alg->fadeInc = 1.0f;
    alg->prevReadPos = 0.0f;
    alg->lastReadInc = 1.0f;
    
    // Initialize coefficient cache (will be calculated on first step)
    alg->lastCalcBase = -1.0f;      // Force recalculation
    alg->lastCalcWidth = -1.0f;
    alg->lastCalcTone = -1.0f;
    alg->lastCalcResonance = -1.0f;
    alg->coeffsInitialized = false;
    
    alg->apdLfoPhase = 0.0f;

    // Setup APD Pointers (at end of main buffer)
    // Main buffer is maxTime seconds * sampleRate frames * 2 channels (interleaved)
    float* apdBase = alg->audioBuffer + (maxTime * sampleRate * 2);
    alg->apd1L.buffer = apdBase; alg->apd1L.size = kApdSize1; alg->apd1L.head = 0;
    apdBase += kApdSize1;
    alg->apd2L.buffer = apdBase; alg->apd2L.size = kApdSize2; alg->apd2L.head = 0;
    apdBase += kApdSize2;
    alg->apd1R.buffer = apdBase; alg->apd1R.size = kApdSize1; alg->apd1R.head = 0;
    apdBase += kApdSize1;
    alg->apd2R.buffer = apdBase; alg->apd2R.size = kApdSize2; alg->apd2R.head = 0;
    
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
        case kParam_Feedback: 
            if (self->v[p] <= -60) pThis->feedback = 0.0f; // True silence at min
            else pThis->feedback = powf(10.0f, self->v[p] / 20.0f); 
            break;
        case kParam_Resonance: {
            float val = self->v[p] / 100.0f;
            pThis->resonance = val * val;
            break;
        }
        case kParam_PingPong: pThis->pingPong = (self->v[p] > 0.5f); break;
        case kParam_LoopMode: pThis->loopMode = (int)self->v[p]; break;
        case kParam_Sync:   pThis->sync = (int)self->v[p]; break;
        case kParam_MidiChannel: pThis->midiChannel = (int)self->v[p]; break;
        case kParam_Tempo:  pThis->tempo = (float)self->v[p]; break;
        case kParam_BitDepth: pThis->bitDepth = (int)self->v[p]; break;
        case kParam_TapeSat: {
            float val = self->v[p] / 100.0f;
            pThis->tapeSat = val * val;
            break;
        }
        case kParam_LoFiSR: {
            float val = self->v[p] / 100.0f;
            pThis->lofiSR = val * val;
            break;
        }
        case kParam_Base: pThis->base = self->v[p] / 127.0f; break;
        case kParam_Width: pThis->width = self->v[p] / 127.0f; break;
        case kParam_Tone: pThis->tone = self->v[p] / 100.0f; break;
        case kParam_Diffusion: {
            float val = self->v[p] / 100.0f;
            pThis->diffusion = val * val;
            break;
        }
        case kParam_Drift: pThis->drift = self->v[p] / 100.0f; break;
        default: break;
    }
}

void midiRealtime(_NT_algorithm* self, uint8_t byte) {
    MrFreezeAlgorithm* pThis = (MrFreezeAlgorithm*)self;
    if (byte == 0xF8) { // Timing Clock (24 PPQN)
        // Accumulate frames across pulses
        pThis->midiClockAccumFrames += pThis->framesSinceLastMidiClock;
        pThis->midiClockPulseCount++;
        
        // Calculate instantaneous BPM for responsive display (heavily smoothed)
        if (pThis->framesSinceLastMidiClock > 0) {
            float seconds = pThis->framesSinceLastMidiClock / (float)NT_globals.sampleRate;
            if (seconds > 0.001f && seconds < 0.5f) { // Sanity check
                float instantBpm = 60.0f / (seconds * 24.0f);
                pThis->detectedMidiBpm = pThis->detectedMidiBpm * 0.95f + instantBpm * 0.05f;
            }
        }
        
        // After 24 pulses (1 beat), calculate stable BPM
        if (pThis->midiClockPulseCount >= 24) {
            float totalSeconds = pThis->midiClockAccumFrames / (float)NT_globals.sampleRate;
            if (totalSeconds > 0.1f && totalSeconds < 4.0f) { // Sanity: 15-600 BPM range
                float beatBpm = 60.0f / totalSeconds;
                // Smooth the stable BPM slightly for gradual tempo changes
                pThis->stableMidiBpm = pThis->stableMidiBpm * 0.7f + beatBpm * 0.3f;
            }
            pThis->midiClockPulseCount = 0;
            pThis->midiClockAccumFrames = 0;
        }
        
        pThis->framesSinceLastMidiClock = 0;
    }
    else if (byte == 0xFA || byte == 0xFB) { // Start or Continue
        // Reset accumulator on transport start
        pThis->midiClockPulseCount = 0;
        pThis->midiClockAccumFrames = 0;
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

    // Filter Coefficients - cached to avoid recalculating every step
    float sr = (float)NT_globals.sampleRate;
    
    // Check if any filter parameters have changed (use threshold to avoid floating point noise)
    bool needRecalcFilters = !pThis->coeffsInitialized ||
        my_abs(pThis->smoothedBase - pThis->lastCalcBase) > 0.001f ||
        my_abs(pThis->smoothedWidth - pThis->lastCalcWidth) > 0.001f ||
        my_abs(pThis->resonance - pThis->lastCalcResonance) > 0.001f;
    
    bool needRecalcTilt = !pThis->coeffsInitialized ||
        my_abs(pThis->smoothedTone - pThis->lastCalcTone) > 0.001f;
    
    if (needRecalcFilters) {
        // Q range: 0.4 (gentle roll-off) to ~4 (resonant but feedback-safe)
        // 0.4 = very gentle slope, 0.707 = Butterworth (flat), higher = resonant peak
        // Original was 0.707 to 14 which was too aggressive for feedback loops
        float Q = 0.4f + (pThis->resonance * 3.6f);
        
        // Map Base (0-1) to Freq Norm (20Hz - 20kHz approx)
        float fBase = 0.00025f * getLinearInterp(pThis->smoothedBase, kPow1000Table);
        float widthVal = pThis->smoothedBase + (pThis->smoothedWidth * (1.0f - pThis->smoothedBase));
        float fWidth = 0.00025f * getLinearInterp(widthVal, kPow1000Table);
        
        // Clamp Width to Tape HF Roll-off (12kHz)
        float maxFWidth = 12000.0f / sr;
        if (fWidth > maxFWidth) fWidth = maxFWidth;
        
        calcFilterCoeffs(kFilterHPF, fBase, Q, pThis->cachedHpfCoeffs);
        calcFilterCoeffs(kFilterLPF, fWidth, Q, pThis->cachedLpfCoeffs);
        calcFilterCoeffs(kFilterHPF, 20.0f / sr, 0.707f, pThis->cachedDcCoeffs);
        
        pThis->lastCalcBase = pThis->smoothedBase;
        pThis->lastCalcWidth = pThis->smoothedWidth;
        pThis->lastCalcResonance = pThis->resonance;
    }
    
    if (needRecalcTilt) {
        calcTiltCoeffs(pThis->smoothedTone, sr, pThis->cachedTiltCoeffs);
        pThis->lastCalcTone = pThis->smoothedTone;
    }
    
    pThis->coeffsInitialized = true;
    
    // Use cached coefficients
    float* hpfCoeffs = pThis->cachedHpfCoeffs;
    float* lpfCoeffs = pThis->cachedLpfCoeffs;
    float* dcCoeffs = pThis->cachedDcCoeffs;
    float* tiltCoeffs = pThis->cachedTiltCoeffs;

    // Update Tape Coeffs if SR changed
    if (sr != pThis->lastSampleRate || pThis->tapeSat != pThis->lastTapeSat) {
        pThis->lastSampleRate = sr;
        pThis->lastTapeSat = pThis->tapeSat;
    }

    // Limiter constants (Sample-rate independent)
    float lookAheadMs = 2.0f;
    uint32_t lookAheadSamples = (uint32_t)(sr * (lookAheadMs * 0.001f));
    float atkCoeff = 1.0f - expf(-1.0f / (sr * 0.001f)); // 1ms Attack
    float relCoeff = 1.0f - expf(-1.0f / (sr * 0.020f)); // 20ms Release
    const float kLimThreshold = 0.99f; // -0.1 dB

    // Determine BPM
    float bpm = 120.0f;
    if (pThis->sync == 1 && clk) { // Sync: Clock
        // External Clock Detector with multi-pulse averaging
        for(int i=0; i<numFrames; ++i) {
            float s = clk[i];
            if (s > 2.0f && pThis->lastClockSample <= 2.0f) { // Rising edge
                if (pThis->framesSinceLastClock > 0) {
                    float seconds = pThis->framesSinceLastClock / (float)NT_globals.sampleRate;
                    if (seconds > 0.01f && seconds < 4.0f) { // Sanity: 15-6000 BPM range
                        // Instantaneous BPM for display (smoothed)
                        float instantBpm = 60.0f / seconds;
                        pThis->detectedBpm = pThis->detectedBpm * 0.8f + instantBpm * 0.2f;
                        
                        // Accumulate for stable BPM calculation
                        pThis->extClockAccumFrames += pThis->framesSinceLastClock;
                        pThis->extClockPulseCount++;
                        
                        // After 4 pulses, calculate stable BPM (1 bar at 4/4)
                        if (pThis->extClockPulseCount >= 4) {
                            float totalSeconds = pThis->extClockAccumFrames / (float)NT_globals.sampleRate;
                            if (totalSeconds > 0.1f) {
                                float barBpm = (60.0f * 4.0f) / totalSeconds; // 4 beats
                                pThis->stableExtBpm = pThis->stableExtBpm * 0.7f + barBpm * 0.3f;
                            }
                            pThis->extClockPulseCount = 0;
                            pThis->extClockAccumFrames = 0;
                        }
                    }
                }
                pThis->framesSinceLastClock = 0;
            }
            pThis->framesSinceLastClock++;
            pThis->lastClockSample = s;
        }
        // Use stable BPM for actual delay calculation
        bpm = pThis->stableExtBpm;
    } else if (pThis->sync == 2) { // Sync: MIDI
        // Use stable BPM (averaged over 1 beat) for actual delay calculation
        bpm = pThis->stableMidiBpm;
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
    float rawDelayLen = samplesPerBeat * beats;
    if (rawDelayLen < 1.0f) rawDelayLen = 1.0f;
    if (rawDelayLen > (float)pThis->bufferSize) rawDelayLen = (float)pThis->bufferSize;
    
    // Detect large jumps BEFORE smoothing (e.g., division changes)
    // Compare raw target against current smoothed target
    float jumpDiff = my_abs(rawDelayLen - pThis->smoothedDelayTarget);
    float jumpThreshold = (float)NT_globals.sampleRate * 0.025f; // 25ms = intentional change
    
    if (pThis->smoothedDelayTarget < 1.0f) {
        // First run - initialize directly
        pThis->smoothedDelayTarget = rawDelayLen;
    } else if (jumpDiff > jumpThreshold) {
        // Large change detected (division change, tempo jump, etc.)
        // Don't smooth - let the per-sample jump detection handle crossfade
        pThis->smoothedDelayTarget = rawDelayLen;
    } else {
        // Small change (clock jitter) - apply hysteresis and slow smoothing
        float jitterThreshold = pThis->smoothedDelayTarget * 0.005f; // 0.5% threshold
        if (jitterThreshold < 1.0f) jitterThreshold = 1.0f;
        
        if (jumpDiff > jitterThreshold) {
            // Update target with slow interpolation to prevent wobble
            if (pThis->sync == 1 || pThis->sync == 2) {
                // External clock or MIDI: very slow interpolation
                pThis->smoothedDelayTarget = pThis->smoothedDelayTarget * 0.9995f + rawDelayLen * 0.0005f;
            } else {
                // Internal tempo: faster response for manual adjustments
                pThis->smoothedDelayTarget = pThis->smoothedDelayTarget * 0.99f + rawDelayLen * 0.01f;
            }
        }
        // Below jitter threshold: don't update at all (hysteresis)
    }
    
    uint32_t delayLen = (uint32_t)(pThis->smoothedDelayTarget + 0.5f);
    if (delayLen < 1) delayLen = 1;
    if (delayLen > pThis->bufferSize) delayLen = pThis->bufferSize;
    
    pThis->currentDelayLen = delayLen;

    // Calculate Window Size for Dry/Wet Fade
    uint32_t W_fade = (uint32_t)(delayLen * (pThis->crossfade / 100.0f));
    if (W_fade < 1) W_fade = 1;
    float dryFadeStep = 1.0f / (float)W_fade;

    pThis->wasFrozen = pThis->freeze;

    // --- Global Drift Calculation (Decimated Block Rate) ---
    pThis->driftUpdateCounter -= numFrames;
    if (pThis->driftUpdateCounter <= 0) {
        pThis->driftUpdateCounter = 64; // Update every 64 samples
        
        float driftAmount = pThis->drift;
        if (driftAmount > 0.0f) {
            // Rate scales with drift amount (higher = faster movement)
            float rate = 0.002f + (driftAmount * 0.01f); 
            
            // Update Filtered Noise (L/R decorrelated)
            float rawL = (getNoise(pThis->rngState) - 0.5f) * 2.0f;
            float rawR = (getNoise(pThis->rngState) - 0.5f) * 2.0f;
            
            // Correlate: Mix 20% of L into R for cohesion (simulating shared PSU sag)
            float nL = rawL;
            float nR = (rawR * 0.8f) + (rawL * 0.2f);
            
            pThis->driftNoiseL += (nL - pThis->driftNoiseL) * rate;
            pThis->driftNoiseR += (nR - pThis->driftNoiseR) * rate;
            
            // 1. Delay Length (High Sensitivity): Scale with delay length (~0.5% max)
            pThis->currentDriftDelay = pThis->driftNoiseL * driftAmount * pThis->smoothedDelayLen * 0.005f; 
            
            // 2. Width (Medium Sensitivity): Only if not at default (1.0)
            if (pThis->width < 0.99f) {
                pThis->currentDriftWidth = pThis->driftNoiseR * (1.0f - pThis->width) * driftAmount * 0.5f;
            } else {
                pThis->currentDriftWidth = 0.0f;
            }

            // 3. Base (Medium Sensitivity): Only if not at default (0)
            if (pThis->base > 0.01f) {
                pThis->currentDriftBase = pThis->driftNoiseR * pThis->base * driftAmount * 0.1f;
            } else {
                pThis->currentDriftBase = 0.0f;
            }
            
            // 4. Tone (Medium Sensitivity): Only if not at default (0.5)
            if (my_abs(pThis->tone - 0.5f) > 0.01f) {
                pThis->currentDriftTone = pThis->driftNoiseR * (pThis->tone - 0.5f) * driftAmount * 0.2f;
            } else {
                pThis->currentDriftTone = 0.0f;
            }
            
            // 4. Lo-Fi SR & Diffusion (Low Sensitivity)
            if (pThis->lofiSR > 0.01f) pThis->currentDriftLofi = pThis->driftNoiseL * pThis->lofiSR * driftAmount * 0.2f;
            else pThis->currentDriftLofi = 0.0f;
            
            if (pThis->diffusion > 0.01f) pThis->currentDriftDiff = pThis->driftNoiseR * pThis->diffusion * driftAmount * 0.2f;
            else pThis->currentDriftDiff = 0.0f;

            // 4. Feedback (Low Sensitivity)
            if (pThis->feedback > 0.001f) pThis->currentDriftFeedback = pThis->driftNoiseR * pThis->feedback * driftAmount * 0.1f;
            else pThis->currentDriftFeedback = 0.0f;
        } else {
            pThis->currentDriftDelay = 0.0f;
            pThis->currentDriftWidth = 0.0f;
            pThis->currentDriftBase = 0.0f;
            pThis->currentDriftTone = 0.0f;
            pThis->currentDriftLofi = 0.0f;
            pThis->currentDriftDiff = 0.0f;
            pThis->currentDriftFeedback = 0.0f;
        }
    }

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

        // Smoothing
        float currentSlew = 0.002f;
        if (pThis->drift > 0.0f) {
            // Modulate slew rate: +/- 50% at max drift to mimic organic component lag
            currentSlew *= (1.0f + (pThis->driftNoiseL * pThis->drift * 0.5f));
            if (currentSlew < 0.0005f) currentSlew = 0.0005f;
        }

        float targetBase = my_max(0.0f, my_min(1.0f, pThis->base + pThis->currentDriftBase));
        pThis->smoothedBase += (targetBase - pThis->smoothedBase) * currentSlew;
        float targetWidth = my_max(0.0f, my_min(1.0f, pThis->width + pThis->currentDriftWidth));
        pThis->smoothedWidth += (targetWidth - pThis->smoothedWidth) * currentSlew;
        
        float targetTone = my_max(0.0f, my_min(1.0f, pThis->tone + pThis->currentDriftTone));
        pThis->smoothedTone += (targetTone - pThis->smoothedTone) * currentSlew;
        
        float targetLofi = my_max(0.0f, my_min(1.0f, pThis->lofiSR + pThis->currentDriftLofi));
        pThis->smoothedLofiSR += (targetLofi - pThis->smoothedLofiSR) * currentSlew;
        
        float targetDiff = my_max(0.0f, my_min(1.0f, pThis->diffusion + pThis->currentDriftDiff));
        pThis->smoothedDiffusion += (targetDiff - pThis->smoothedDiffusion) * currentSlew;

        float targetFb = my_max(0.0f, pThis->feedback + pThis->currentDriftFeedback);
        pThis->smoothedFeedback += (targetFb - pThis->smoothedFeedback) * currentSlew;

        // 1. Detect Macro Jumps (e.g., changing divisions)
        float targetLen = (float)pThis->currentDelayLen;
        float jumpDiff = my_abs(targetLen - pThis->fadeTargetLen);
        
        if (pThis->loopMode != pThis->lastLoopMode) {
            jumpDiff = 10000.0f; // Force jump
            pThis->lastLoopMode = pThis->loopMode;
        }

        // 25ms threshold to prevent jitter resets (sample-rate dependent)
        float jumpThreshold = (float)NT_globals.sampleRate * 0.025f;
        if (jumpDiff > jumpThreshold) {
            // Capture state for fade out (Linear Continuation)
            pThis->fadeReadPos = pThis->prevReadPos;
            pThis->fadeInc = pThis->lastReadInc;
            // Safety clamp for velocity
            if (pThis->fadeInc > 2.0f) pThis->fadeInc = 2.0f;
            if (pThis->fadeInc < -2.0f) pThis->fadeInc = -2.0f;

            pThis->fadeOldLen = pThis->smoothedDelayLen; // Kept for reference
            pThis->fadeTargetLen = targetLen;
            pThis->smoothedDelayLen = targetLen; // Instantly jump to the new length
            pThis->fadePhase = 0.0f; // Reset crossfade progress
            pThis->reversePhase = 0.0f;
        } else {
            pThis->fadeTargetLen = targetLen;
        }

        // 2. Slew for Micro-corrections
        // Use 0.001f for standard tracking to stay locked to grid
        pThis->smoothedDelayLen += ((pThis->fadeTargetLen + pThis->currentDriftDelay) - pThis->smoothedDelayLen) * 0.001f;

        // 3. Read Main Tap (Tap A)
        float L = pThis->smoothedDelayLen;
        float doubleL = L * 2.0f;

        // 1. Calculate Speed (with Drift)
        float phaseSpeed = 1.0f;
        if (pThis->drift > 0.0f) {
            // Modulate speed with drift noise (approx +/- 1%)
            phaseSpeed += pThis->driftNoiseL * pThis->drift * 0.01f;
        }

        // 2. Increment Phase
        pThis->reversePhase += phaseSpeed;

        // 3. Wrap Phase
        if (pThis->loopMode == 2) { // Ping-Pong
            while (pThis->reversePhase >= doubleL) pThis->reversePhase -= doubleL;
            if (pThis->reversePhase < 0.0f) pThis->reversePhase += doubleL;
        } else { // Fwd or Rev
            while (pThis->reversePhase >= L) pThis->reversePhase -= L;
            if (pThis->reversePhase < 0.0f) pThis->reversePhase += L;
        }

        // 4. Map to Offset
        float pingPongOffset = 0.0f; // Default for Mode 0 (Standard Delay)
        float secondaryOffset = -9999.0f; // Flag for no secondary
        float bounceFade = 0.0f;
        
        // Calculate Fade Length based on parameter (0-50% of loop)
        // Ensure minimum of 4 samples to prevent division by zero
        float fadeLen = my_max(4.0f, L * (pThis->crossfade * 0.01f));

        if (pThis->loopMode == 2) { // Ping-Pong
            // Top Turnaround (L) - crossfade region when reversing
            if (pThis->reversePhase >= L && pThis->reversePhase < L + fadeLen) {
                pingPongOffset = doubleL - pThis->reversePhase; // Main (Reverse)
                secondaryOffset = pThis->reversePhase - L;      // Secondary (Forward Overshoot amount)
                bounceFade = (pThis->reversePhase - L) / fadeLen; // 0 (Old) -> 1 (New)
            }
            // Bottom Turnaround (0) - crossfade region when reversing
            else if (pThis->reversePhase < fadeLen) {
                pingPongOffset = pThis->reversePhase;           // Main (Forward)
                secondaryOffset = -pThis->reversePhase;         // Secondary (Reverse Overshoot amount, negative)
                bounceFade = pThis->reversePhase / fadeLen;       // 0 (Old) -> 1 (New)
            }
            // Forward section (middle) - playing forward from fadeLen to L
            else if (pThis->reversePhase < L) {
                pingPongOffset = pThis->reversePhase; // Forward playback
            }
            // Reverse section (middle) - playing reverse from L+fadeLen to doubleL
            else {
                // reversePhase >= L + fadeLen && reversePhase < doubleL
                pingPongOffset = doubleL - pThis->reversePhase; // Reverse playback
            }
        } 
        else if (pThis->loopMode == 1) { // Reverse
            // In reverse, reversePhase goes 0->L, but we want to read backwards
            // So we use reversePhase directly as the offset from anchor-L
            pingPongOffset = pThis->reversePhase; // 0 -> L
            // Loop Wrap-Around: when reversePhase wraps from L->0, we're at the start again
            // Crossfade at the end (when reversePhase is near L, pingPongOffset is near L)
            if (pingPongOffset > L - fadeLen) {
                secondaryOffset = pingPongOffset - L; // Negative overshoot
                bounceFade = (pingPongOffset - (L - fadeLen)) / fadeLen; // 0 (Old) -> 1 (New)
            }
        } 
        // Mode 0 (Fwd): pingPongOffset remains 0.0f (Static)

        // 5. Calculate Read Position
        float anchor = (float)pThis->writeHead;
        float readPos;
        if (pThis->loopMode == 1) { // Reverse mode
            // For reverse playback, readPos must DECREASE while anchor INCREASES
            // Since anchor increases by 1 per sample, we need offset to increase by 2
            // Formula: readPos = anchor - 1 - 2*reversePhase
            // Sample 0: anchor=A, phase=0 → readPos = A-1 (newest)
            // Sample 1: anchor=A+1, phase=1 → readPos = A+1-1-2 = A-2
            // Sample k: anchor=A+k, phase=k → readPos = A+k-1-2k = A-1-k
            // At phase=L-1: readPos = A-1-(L-1) = A-L (oldest)
            readPos = anchor - 1.0f - 2.0f * pingPongOffset;
        } else if (pThis->loopMode == 2) { // Ping-Pong
            if (pThis->reversePhase >= L) {
                // Reverse section: must start where forward ended and go backwards
                // Forward ended at readPos = A-1 when anchor was at A+L-1
                // Now anchor is at A+L (when reversePhase=L), we need readPos = A-1
                // Formula: readPos = anchor - L - 1 - 2*(reversePhase - L)
                // At reversePhase=L: anchor=A+L, readPos = A+L-L-1-0 = A-1 (newest)
                // At reversePhase=2L-1: anchor=A+2L-1, readPos = A+2L-1-L-1-2*(L-1) = A-L (oldest)
                float reverseSectionPhase = pThis->reversePhase - L;
                readPos = anchor - L - 1.0f - 2.0f * reverseSectionPhase;
            } else {
                // Forward section: standard delay behavior (readPos = anchor - L)
                // This keeps readPos at a fixed distance from write head
                readPos = anchor - L;
            }
        } else {
            // Forward mode: standard delay (read from anchor - L)
            readPos = anchor - L;
        }

        // Wrap around buffer
        readPos = wrapBufferPos(readPos, pThis->bufferSize);

        uint32_t idxA = (uint32_t)readPos;
        uint32_t idxB = (idxA + 1) % pThis->bufferSize;
        float fraction = readPos - (float)idxA;

        float delayL = pThis->audioBuffer[idxA * 2] * (1.0f - fraction) + 
                       pThis->audioBuffer[idxB * 2] * fraction;
        float delayR = pThis->audioBuffer[idxA * 2 + 1] * (1.0f - fraction) + 
                       pThis->audioBuffer[idxB * 2 + 1] * fraction;

        // Apply Bounce Crossfade if active
        if (secondaryOffset > -9000.0f) {
            float readPos2;
            if (pThis->loopMode == 1) { // Reverse mode
                // Secondary represents the NEW loop starting from newest sample
                // It should read backward from anchor-1 as the crossfade progresses
                // crossfadeProgress = pingPongOffset - (L - fadeLen) = secondaryOffset + fadeLen
                // readPos2 = anchor - 1 - 2*crossfadeProgress
                float crossfadeProgress = secondaryOffset + fadeLen;
                readPos2 = anchor - 1.0f - 2.0f * crossfadeProgress;
            } else if (pThis->loopMode == 2) { // Ping-Pong
                if (pThis->reversePhase >= L) {
                    // Top turnaround: entering reverse from forward
                    // Secondary is forward continuation - just keep using forward formula
                    // Forward reads at readPos = anchor - L, so secondary continues this
                    readPos2 = anchor - L;
                } else {
                    // Bottom turnaround: entering forward from reverse
                    // Secondary is reverse continuation
                    // Need to continue reading backward from where reverse section ended
                    // Reverse section ended at position anchor-L-1 when reversePhase was 2L-1
                    // Now reversePhase has wrapped to 0, we continue backward from there
                    float crossfadeProgress = -secondaryOffset; // = reversePhase (0 to fadeLen)
                    // At reversePhase=0 (crossfadeProgress=0): read from anchor-L-1
                    // As crossfadeProgress increases, continue backward (subtract 2*progress for reverse speed)
                    readPos2 = anchor - L - 1.0f - 2.0f * crossfadeProgress;
                }
            } else {
                readPos2 = anchor - L + secondaryOffset;
            }
            readPos2 = wrapBufferPos(readPos2, pThis->bufferSize);

            uint32_t idxA2 = (uint32_t)readPos2;
            uint32_t idxB2 = (idxA2 + 1) % pThis->bufferSize;
            float fraction2 = readPos2 - (float)idxA2;

            float delayL2 = pThis->audioBuffer[idxA2 * 2] * (1.0f - fraction2) + 
                            pThis->audioBuffer[idxB2 * 2] * fraction2;
            float delayR2 = pThis->audioBuffer[idxA2 * 2 + 1] * (1.0f - fraction2) + 
                            pThis->audioBuffer[idxB2 * 2 + 1] * fraction2;

            // Crossfade direction depends on mode:
            // - Reverse mode: Main = current loop END, Secondary = new loop START
            //   Want: fade FROM main (old) TO secondary (new)
            // - Ping-pong turnarounds: Main = NEW direction starting, Secondary = OLD direction continuing  
            //   Want: fade FROM secondary (old) TO main (new)
            float gMain, gSec;
            if (pThis->loopMode == 2) { // Ping-Pong: fade secondary → main
                gMain = getSineInterp(bounceFade);           // 0→1 (fade in new direction)
                gSec = getSineInterp(1.0f - bounceFade);     // 1→0 (fade out old direction)
            } else { // Reverse: fade main → secondary
                gMain = getSineInterp(1.0f - bounceFade);    // 1→0 (fade out current loop)
                gSec = getSineInterp(bounceFade);            // 0→1 (fade in new loop)
            }

            delayL = (delayL * gMain) + (delayL2 * gSec);
            delayR = (delayR * gMain) + (delayR2 * gSec);
        }

        // 4. Clean Crossfade for Jumps
        if (pThis->fadePhase < 1.0f) {
            pThis->fadePhase += 1.0f / (sr * 0.025f); // 25ms fade
            if (pThis->fadePhase > 1.0f) pThis->fadePhase = 1.0f;
            
            pThis->fadeReadPos += pThis->fadeInc;
            pThis->fadeReadPos = wrapBufferPos(pThis->fadeReadPos, pThis->bufferSize);
            
            uint32_t idxA_Old = (uint32_t)pThis->fadeReadPos;
            uint32_t idxB_Old = (idxA_Old + 1) % pThis->bufferSize;
            float fractionOld = pThis->fadeReadPos - (float)idxA_Old;
            
            float delayL_Old = pThis->audioBuffer[idxA_Old * 2] * (1.0f - fractionOld) + 
                               pThis->audioBuffer[idxB_Old * 2] * fractionOld;
            float delayR_Old = pThis->audioBuffer[idxA_Old * 2 + 1] * (1.0f - fractionOld) + 
                               pThis->audioBuffer[idxB_Old * 2 + 1] * fractionOld;
            
            float gainOld = getSineInterp(1.0f - pThis->fadePhase);
            float gainNew = getSineInterp(pThis->fadePhase);
            
            delayL = (delayL_Old * gainOld) + (delayL * gainNew);
            delayR = (delayR_Old * gainOld) + (delayR * gainNew);
        }

        // 3. Process Effects on the Wet Signal (Buffer Read)
        // Lo-Fi Sample Rate Reduction (Hybrid approach: better filter + cosine interp)
        if (pThis->smoothedLofiSR > 0.001f) {
            // 1. Calculate Step Size (how fast we advance through samples)
            // Use sqrt curve for more usable range (spreads control across full 0-100%)
            // stepSize: 1.0 (no reduction) to 0.02 (50x reduction, feedback-safe)
            float lofiAmount = pThis->smoothedLofiSR;
            float curve = lofiAmount * lofiAmount; // Quadratic curve for smoother feel
            float stepSize = 1.0f - (curve * 0.98f); // 1.0 down to 0.02

            // Subtle jitter for analog clock feel
            float jitter = (getNoise(pThis->rngState) - 0.5f) * 0.01f * stepSize;
            stepSize += jitter;
            
            // 2. Gentle Pre-Filter (anti-aliasing without excessive darkening)
            // Less aggressive filtering - let the sample rate reduction do the work
            // Higher minimum alpha = brighter sound even at extreme settings
            float alpha = my_max(0.08f, my_min(0.6f, stepSize * 0.8f));
            // First pole
            pThis->lofiPreFilterL += alpha * (delayL - pThis->lofiPreFilterL);
            pThis->lofiPreFilterR += alpha * (delayR - pThis->lofiPreFilterR);
            // Second pole (cascaded for steeper rolloff)
            pThis->lofiPreFilter2L += alpha * (pThis->lofiPreFilterL - pThis->lofiPreFilter2L);
            pThis->lofiPreFilter2R += alpha * (pThis->lofiPreFilterR - pThis->lofiPreFilter2R);
            float inL = pThis->lofiPreFilter2L;
            float inR = pThis->lofiPreFilter2R;

            // 3. Sample-and-Hold with accumulator
            pThis->lofiCounter += stepSize;
            if (pThis->lofiCounter >= 1.0f) {
                pThis->lofiCounter -= 1.0f;
                pThis->prevLofiSampleL = pThis->lastLofiSampleL;
                pThis->prevLofiSampleR = pThis->lastLofiSampleR;
                pThis->lastLofiSampleL = inL;
                pThis->lastLofiSampleR = inR;
            }

            // 4. Cosine Interpolation (smooth S-curve between samples)
            float t = getCosineInterp(pThis->lofiCounter);
            delayL = pThis->prevLofiSampleL + (pThis->lastLofiSampleL - pThis->prevLofiSampleL) * t;
            delayR = pThis->prevLofiSampleR + (pThis->lastLofiSampleR - pThis->prevLofiSampleR) * t;
        }

        // Bit Crush
        if (pThis->bitDepth < 24) {
            delayL = bitCrush(delayL, pThis->bitDepth, pThis->rngState);
            delayR = bitCrush(delayR, pThis->bitDepth, pThis->rngState);
        }

        // Tilt EQ - bypass when tone is centered (0.5)
        if (my_abs(pThis->smoothedTone - 0.5f) < 0.005f) {
            // Reset filter state when bypassed to prevent clicks when re-enabled
            pThis->tiltStateL = {};
            pThis->tiltStateR = {};
        } else {
            delayL = processBiquad(delayL, pThis->tiltStateL, tiltCoeffs);
            delayR = processBiquad(delayR, pThis->tiltStateR, tiltCoeffs);
        }

        // Filters (Base/Width)
        // HPF Bypass - when base frequency is at minimum (no high-pass filtering)
        if (pThis->smoothedBase <= 0.001f) {
            pThis->hpfStateL = {};
            pThis->hpfStateR = {};
        } else {
            delayL = processBiquad(delayL, pThis->hpfStateL, hpfCoeffs);
            delayR = processBiquad(delayR, pThis->hpfStateR, hpfCoeffs);
        }

        // LPF Bypass - when width is at maximum (no low-pass filtering beyond tape roll-off)
        // Width >= 0.999 means LPF frequency is at 12kHz ceiling, effectively bypassed for user control
        if (pThis->smoothedWidth >= 0.999f && pThis->smoothedBase <= 0.001f) {
            // Full bypass when both base and width are at extremes (no filtering at all)
            pThis->lpfStateL = {};
            pThis->lpfStateR = {};
        } else {
            delayL = processBiquad(delayL, pThis->lpfStateL, lpfCoeffs);
            delayR = processBiquad(delayR, pThis->lpfStateR, lpfCoeffs);
        }

        // Tape Saturation (Always On - serves as soft clipper even at 0%)
        float tapeAmount = pThis->tapeSat;
        float drive = 1.0f + (tapeAmount * 0.25f);
        delayL = processTape(delayL, drive, tapeAmount, pThis->tapeL, dcCoeffs);
        delayR = processTape(delayR, drive, tapeAmount, pThis->tapeR, dcCoeffs);

        // Diffusion (Nested All-Pass) - bypass when amount is zero
        if (pThis->smoothedDiffusion > 0.001f) {
            // Slow LFO for modulation (approx 0.2Hz)
            pThis->apdLfoPhase += 0.000013f; 
            if (pThis->apdLfoPhase > 6.283f) pThis->apdLfoPhase -= 6.283f;

            float p = pThis->apdLfoPhase * 0.63661977f; // 2/pi
            float sineVal;
            if (p <= 1.0f) sineVal = getSineInterp(p);
            else if (p <= 2.0f) sineVal = getSineInterp(2.0f - p);
            else if (p <= 3.0f) sineVal = -getSineInterp(p - 2.0f);
            else sineVal = -getSineInterp(4.0f - p);
            float mod = sineVal * 20.0f; // +/- 20 samples

            // Calculate modulated gain
            float apGain = 0.5f;
            if (pThis->drift > 0.0f) {
                // Fluctuate between 0.25 and 0.5 based on drift noise
                float targetG = 0.375f + (pThis->driftNoiseL * 0.125f);
                apGain = (0.5f * (1.0f - pThis->drift)) + (targetG * pThis->drift);
            }

            float diffL = processNestedAllPass(delayL, pThis->apd1L, pThis->apd2L, mod, -mod, apGain);
            float diffR = processNestedAllPass(delayR, pThis->apd1R, pThis->apd2R, mod * 0.7f, -mod * 0.7f, apGain);

            // 5% Cross-feed
            float cfL = diffL + 0.05f * diffR;
            float cfR = diffR + 0.05f * diffL;

            delayL = (delayL * (1.0f - pThis->smoothedDiffusion)) + (cfL * pThis->smoothedDiffusion);
            delayR = (delayR * (1.0f - pThis->smoothedDiffusion)) + (cfR * pThis->smoothedDiffusion);
        } else {
            // Reset all-pass state when diffusion is bypassed
            pThis->apd1L.head = 0;
            pThis->apd2L.head = 0;
            pThis->apd1R.head = 0;
            pThis->apd2R.head = 0;
        }

        // --- Transparent Look-Ahead Limiter ---
        // 1. Look-Ahead Detection
        float futureReadPos = readPos + (float)lookAheadSamples;
        futureReadPos = wrapBufferPos(futureReadPos, pThis->bufferSize);

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
        float wetL = delayL;
        float wetR = delayR;

        float gainDry = getSineInterp(pThis->currentDryGain);
        float gainWet = getSineInterp(1.0f - pThis->currentDryGain);
        float mixL = (inSampleL * gainDry) + (wetL * gainWet);
        float mixR = (inSampleR * gainDry) + (wetR * gainWet);

        if (replaceL) outL[i] = mixL;
        else          outL[i] += mixL;

        if (replaceR) outR[i] = mixR;
        else          outR[i] += mixR;

        // 5. Feedback & Write to Buffer
        float fbSourceL = delayL;
        float fbSourceR = delayR;

        if (fbInL) fbSourceL = fbInL[i];
        if (fbInR) fbSourceR = fbInR[i];

        float fbVal = pThis->smoothedFeedback;
        
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

        // Track Read Head Velocity for next Jump
        float diff = readPos - pThis->prevReadPos;
        // Handle buffer wrap
        if (diff < -pThis->bufferSize * 0.5f) diff += (float)pThis->bufferSize;
        if (diff > pThis->bufferSize * 0.5f) diff -= (float)pThis->bufferSize;
        
        pThis->lastReadInc = diff;
        pThis->prevReadPos = readPos;
    }

    // Update MIDI clock counter
    pThis->framesSinceLastMidiClock += numFrames;
}

bool draw(_NT_algorithm* self) {
    MrFreezeAlgorithm* pThis = (MrFreezeAlgorithm*)self;
    char buf[32];
    
    // Layout: y=0-17 reserved for system parameter bar
    // Row 1 (y=18): Feedback, Mode, Division, BPM
    // Waveform (y=28-63): Static 1-division view with playhead
    // Limiter bar at bottom of waveform
    
    // === ROW 1: Status Info (y=18) ===
    // Feedback (left)
    int fb = pThis->v[kParam_Feedback];
    if (fb <= -60) {
        NT_drawText(0, 18, "FB:-inf", 10);
    } else {
        buf[0] = 'F'; buf[1] = 'B'; buf[2] = ':';
        int fbLen = NT_intToString(buf + 3, fb);
        buf[3 + fbLen] = 0;
        NT_drawText(0, 18, buf, 10);
    }
    
    // Loop Mode
    const char* modeStr = "Fwd";
    if (pThis->loopMode == 1) modeStr = "Rev";
    else if (pThis->loopMode == 2) modeStr = "P-P";
    NT_drawText(50, 18, modeStr, pThis->freeze ? 15 : 8);
    
    // Division display (e.g., "1/4", "1/4t", "1/4.")
    static const char* divNames[] = {"1/64","1/32","1/16","1/8","1/4","1/2","1/1","2/1","4/1"};
    int divIdx = pThis->division;
    if (divIdx < 0) divIdx = 0;
    if (divIdx > 8) divIdx = 8;
    
    int pos = 0;
    const char* divName = divNames[divIdx];
    while (*divName && pos < 8) buf[pos++] = *divName++;
    if (pThis->triplet == 1) buf[pos++] = 't';
    else if (pThis->triplet == 2) buf[pos++] = 'T';
    if (pThis->dotted) buf[pos++] = '.';
    buf[pos] = 0;
    NT_drawText(128, 18, buf, 12, kNT_textCentre);
    
    // BPM display (right aligned)
    float currentBpm = pThis->tempo;
    if (pThis->sync == 1) currentBpm = pThis->detectedBpm;
    else if (pThis->sync == 2) currentBpm = pThis->detectedMidiBpm;
    NT_floatToString(buf, currentBpm, 1);
    int len = 0; while (buf[len]) len++;
    buf[len] = 'b'; buf[len+1] = 'p'; buf[len+2] = 'm'; buf[len+3] = 0;
    NT_drawText(255, 18, buf, 12, kNT_textRight);
    
    // === WAVEFORM DISPLAY (y=28 to 63) ===
    int waveTop = 28;
    int waveBot = 63;
    int waveH = waveBot - waveTop;
    int yMid = waveTop + waveH / 2;
    
    // Clear and draw border
    NT_drawShapeI(kNT_rectangle, 0, waveTop, 255, waveBot, 0);
    NT_drawShapeI(kNT_box, 0, waveTop, 255, waveBot, 3);
    
    // Center line (zero crossing reference)
    NT_drawShapeI(kNT_line, 1, yMid, 254, yMid, 2);
    
    float L = pThis->smoothedDelayLen;
    if (L < 1.0f) L = 1.0f;
    
    // Crossfade region width (as fraction of display)
    float fadeWidth = pThis->crossfade * 0.01f;
    int fadePixels = (int)(fadeWidth * 254.0f);
    
    // Draw crossfade regions (shaded areas at boundaries) - STATIC
    if (fadePixels > 2 && pThis->crossfade > 1.0f) {
        // Left crossfade region (loop start)
        NT_drawShapeI(kNT_rectangle, 1, waveTop + 1, 1 + fadePixels, waveBot - 1, 1);
        // Right crossfade region (loop end)
        NT_drawShapeI(kNT_rectangle, 254 - fadePixels, waveTop + 1, 254, waveBot - 1, 1);
    }
    
    // Draw waveform - STATIC view showing last L samples
    // x=1 is oldest (writeHead - L), x=254 is newest (writeHead)
    uint32_t loopStart = (pThis->writeHead + pThis->bufferSize - (uint32_t)L) % pThis->bufferSize;
    int lastY = yMid;
    int waveColor = pThis->freeze ? 7 : 11;
    
    for (int x = 1; x < 255; ++x) {
        float frac = (float)(x - 1) / 253.0f;
        uint32_t idx = (loopStart + (uint32_t)(frac * L)) % pThis->bufferSize;
        float sample = pThis->audioBuffer[idx * 2];
        
        int y = yMid - (int)(sample * (float)(waveH / 2 - 3));
        if (y < waveTop + 1) y = waveTop + 1;
        if (y > waveBot - 2) y = waveBot - 2;
        
        NT_drawShapeI(kNT_line, x - 1, lastY, x, y, waveColor);
        lastY = y;
    }
    
    // Read head position - where in the loop we're currently playing
    // reversePhase is 0 to L (or 0 to 2L for ping-pong)
    float readFrac;
    if (pThis->loopMode == 2) {
        // Ping-pong: phase 0->L is forward, L->2L is reverse
        float phase = pThis->reversePhase;
        float doubleL = L * 2.0f;
        if (phase >= doubleL) phase -= doubleL;
        if (phase < L) {
            readFrac = phase / L; // Forward: 0->1
        } else {
            readFrac = (doubleL - phase) / L; // Reverse: 1->0
        }
    } else if (pThis->loopMode == 1) {
        // Reverse: phase goes 0->L but we read backwards
        readFrac = 1.0f - (pThis->reversePhase / L);
    } else {
        // Forward: read position relative to loop
        readFrac = pThis->reversePhase / L;
    }
    if (readFrac < 0.0f) readFrac = 0.0f;
    if (readFrac > 1.0f) readFrac = 1.0f;
    
    int readX = 1 + (int)(readFrac * 253.0f);
    NT_drawShapeI(kNT_line, readX, waveTop + 1, readX, waveBot - 2, 15);
    
    // Limiter gain reduction bar at bottom of waveform
    if (pThis->limiterGain < 0.995f) {
        float grDb = 20.0f * log10f(my_max(0.001f, pThis->limiterGain));
        float grNorm = my_min(1.0f, my_abs(grDb) / 12.0f);
        int grWidth = (int)(grNorm * 253.0f);
        if (grWidth > 0) {
            NT_drawShapeI(kNT_rectangle, 1, waveBot - 1, 1 + grWidth, waveBot - 1, 14);
        }
        // Show dB value
        NT_floatToString(buf, grDb, 1);
        NT_drawText(254, waveBot - 8, buf, 14, kNT_textRight, kNT_textTiny);
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