# Code Review: MrFreezeNT.cpp

## Executive Summary
This is a sophisticated delay/looper plugin for the disting NT. The code is generally well-structured but contains several logic issues, potential bugs, and areas for improvement.

## Critical Issues

### 1. **Buffer Size Calculation Inconsistency** ⚠️
**Location:** Lines 498, 586, 1031-1034

The `bufferSize` is set to the number of stereo frames (`maxTime * 96000`), but the buffer is interleaved (2 floats per frame). While the indexing math works out correctly (`idxA * 2`), this is confusing and error-prone.

**Issue:** 
- Line 498: `alg->bufferSize = maxTime * 96000;` // Comment says "Stereo frames"
- Line 586: `alg->audioBuffer + (maxTime * 96000 * 2)` // Correctly accounts for 2 floats per frame
- The code uses `idxA * 2` for indexing, which works but is non-obvious

**Recommendation:** Either:
- Store `bufferSize` as the number of samples (floats), or
- Add clear comments explaining the stereo frame convention
- Consider using a helper function for buffer indexing

### 2. **Unused Variable: `freezeAnchor`** 🐛
**Location:** Lines 65, 500

`freezeAnchor` is declared and initialized but never used in the codebase. This suggests incomplete implementation or leftover code.

**Recommendation:** Either implement freeze anchor functionality or remove it.

### 3. **Ping-Pong Logic Incomplete** 🐛
**Location:** Lines 992-1008

In ping-pong mode, the logic only handles the turnaround regions (with crossfade), but doesn't set `pingPongOffset` for the middle sections:
- When `fadeLen <= reversePhase < L`: `pingPongOffset` remains 0.0f (should be `reversePhase` for forward)
- When `L + fadeLen <= reversePhase < doubleL`: `pingPongOffset` remains 0.0f (should be `doubleL - reversePhase` for reverse)

The third `else if` (line 1005) checks `pingPongOffset > L`, but since `pingPongOffset` is only set in the first two conditions (which handle turnarounds), this condition will never be true in the middle sections.

**Current behavior:** The code may work because `pingPongOffset = 0.0f` happens to work for forward playback, but reverse playback in the middle section won't work correctly.

**Recommendation:** Add explicit handling for middle sections:
```cpp
if (pThis->loopMode == 2) { // Ping-Pong
    if (pThis->reversePhase >= L && pThis->reversePhase < L + fadeLen) {
        // Top turnaround with crossfade
        pingPongOffset = doubleL - pThis->reversePhase;
        secondaryOffset = pThis->reversePhase;
        bounceFade = (pThis->reversePhase - L) / fadeLen;
    }
    else if (pThis->reversePhase < fadeLen) {
        // Bottom turnaround with crossfade
        pingPongOffset = pThis->reversePhase;
        secondaryOffset = -pThis->reversePhase;
        bounceFade = pThis->reversePhase / fadeLen;
    }
    else if (pThis->reversePhase < L) {
        // Forward section (middle)
        pingPongOffset = pThis->reversePhase;
    }
    else {
        // Reverse section (middle) - reversePhase >= L + fadeLen
        pingPongOffset = doubleL - pThis->reversePhase;
    }
}
```

### 4. **Hardcoded Sample Rate Assumption** ⚠️ **CRITICAL BUG**
**Location:** Lines 483, 498, 586

The code assumes 96kHz sample rate in buffer size calculations:
- Line 483: `req.dram = (maxTime * 96000 * 2 + kTotalApdSize) * sizeof(float);`
- Line 498: `alg->bufferSize = maxTime * 96000; // Stereo frames`
- Line 586: `alg->audioBuffer + (maxTime * 96000 * 2)` for APD buffer placement

**Issue:** The code uses `NT_globals.sampleRate` elsewhere (lines 660, 737, 746, 780, 819) but hardcodes 96000 for buffer allocation. If the actual sample rate differs (e.g., 48kHz), the buffer will be allocated incorrectly - too large at lower sample rates, potentially too small at higher ones.

**Impact:** 
- At 48kHz: Buffer is 2x larger than needed (wastes memory)
- At 192kHz: Buffer might be too small (potential buffer overrun)

**Recommendation:** 
```cpp
// In calculateRequirements:
float sampleRate = (float)NT_globals.sampleRate;
req.dram = (maxTime * (uint32_t)sampleRate * 2 + kTotalApdSize) * sizeof(float);

// In construct:
float sampleRate = (float)NT_globals.sampleRate;
alg->bufferSize = maxTime * (uint32_t)sampleRate; // Stereo frames
float* apdBase = alg->audioBuffer + (maxTime * (uint32_t)sampleRate * 2);
```

### 5. **Clock Detection Threshold** ⚠️
**Location:** Line 778

```cpp
if (s > 2.0f && pThis->lastClockSample <= 2.0f) { // Rising edge
```

The threshold of 2.0V is arbitrary and may not work for all clock signals (some use 5V, some use 10V).

**Recommendation:** 
- Make threshold configurable or use a more robust detection method
- Consider using a percentage of expected peak or adaptive threshold

## Logic Issues

### 6. **Drift Update Rate Not Sample-Rate Dependent**
**Location:** Line 836

```cpp
pThis->driftUpdateCounter = 64; // Update every 64 samples
```

This is hardcoded to 64 samples regardless of sample rate. At 48kHz this is ~1.3ms, at 96kHz it's ~0.67ms.

**Recommendation:** Either make it sample-rate dependent or document the reasoning for the fixed rate.

### 7. **Division Calculation** ✅
**Location:** Lines 802-812

The division mapping correctly handles all cases including case 8 (16/1). No issue here.

### 8. **MIDI Channel 0 Handling**
**Location:** Line 677

```cpp
if (pThis->midiChannel > 0 && channel != (pThis->midiChannel - 1)) {
    return;
}
```

This correctly treats channel 0 as "all channels" (no filtering), but the parameter definition (line 205) allows 0-16, which means 17 possible values. MIDI channels are 0-15, so value 16 is invalid.

**Recommendation:** Change max to 15, or handle 16 as "all channels" if that's intentional.

### 9. **Reverse Phase Wrapping Logic**
**Location:** Lines 975-981

The wrapping logic uses `while` loops which could theoretically be slow if phase is very far out of range (though unlikely in practice).

**Recommendation:** Use modulo arithmetic or `fmodf()` for cleaner code:
```cpp
pThis->reversePhase = fmodf(pThis->reversePhase, doubleL);
if (pThis->reversePhase < 0.0f) pThis->reversePhase += doubleL;
```

## Quality Issues

### 10. **Magic Numbers**
Throughout the code, there are many magic numbers without explanation:
- Line 778: `2.0f` (clock threshold)
- Line 836: `64` (drift update rate)
- Line 939: `2400.0f` (jump threshold - 25ms at 96kHz)
- Line 1060: `0.025f` (25ms fade time)
- Line 1163: `0.000013f` (LFO phase increment)
- Line 1172: `20.0f` (modulation range)

**Recommendation:** Define named constants with comments explaining their purpose.

### 11. **Inconsistent Smoothing Rates**
**Location:** Lines 906-928

Different parameters use different smoothing approaches:
- Base/Width/Tone use `currentSlew` (0.002f base, modulated by drift)
- Delay length uses 0.001f (line 958)
- LofiSR/Diffusion/Feedback use `currentSlew`

**Recommendation:** Document why different rates are used, or consider making them consistent.

### 12. **Tilt EQ Reset Logic**
**Location:** Lines 1130-1136

The tilt EQ is reset when tone is near 0.5 (neutral), but the threshold `0.005f` is arbitrary.

**Recommendation:** Document why this reset is necessary and if the threshold is optimal.

### 13. **Look-Ahead Limiter Future Read Position**
**Location:** Lines 1195-1204

The limiter reads ahead, but this uses the current `readPos` which may not account for ping-pong or reverse modes correctly.

**Recommendation:** Verify that the future read position calculation works correctly in all loop modes.

## Potential Bugs

### 14. **Buffer Wrap-Around in Read Position Calculation**
**Location:** Lines 1024-1025, 1039-1040, etc.

Multiple places use `while` loops for wrapping:
```cpp
while (readPos < 0.0f) readPos += (float)pThis->bufferSize;
while (readPos >= (float)pThis->bufferSize) readPos -= (float)pThis->bufferSize;
```

If `readPos` is extremely negative or positive (due to a bug), this could loop many times.

**Recommendation:** Use modulo or add bounds checking before wrapping.

### 15. **Secondary Offset Calculation in Reverse Mode**
**Location:** Line 1014

```cpp
secondaryOffset = pingPongOffset - L; // Near 0
```

If `pingPongOffset` is exactly `L`, this gives `0`, but the condition is `pingPongOffset > L - fadeLen`, so `pingPongOffset` could be `L - fadeLen + epsilon`, making `secondaryOffset` negative. This might be intentional for the crossfade, but should be verified.

### 16. **Active Note Count Can Go Negative**
**Location:** Lines 682, 694-698

If a Note Off is received without a corresponding Note On (e.g., after a reset), `activeNoteCount` can go negative, though it's clamped to 0.

**Recommendation:** The clamping is good, but consider logging or handling this edge case more explicitly.

## Performance Issues

### 17. **Clock Detection in Audio Loop**
**Location:** Lines 776-790

Clock detection runs in the audio processing loop for every sample when sync mode is Clock. This could be optimized by only checking on buffer boundaries or using a separate detection routine.

**Recommendation:** Consider moving clock detection to a separate function called less frequently.

### 18. **Repeated Coefficient Calculations**
**Location:** Lines 733-753

Filter coefficients are recalculated every `step()` call, even if parameters haven't changed. This is wasteful.

**Recommendation:** Cache coefficients and only recalculate when relevant parameters change.

### 19. **Sine Table Lookup in Diffusion LFO**
**Location:** Lines 1166-1172

The LFO uses a complex sine approximation that could be simplified or use a proper sine function if performance allows.

## Code Organization

### 20. **Long `step()` Function**
**Location:** Lines 702-1277

The `step()` function is 575 lines long, making it hard to maintain and test.

**Recommendation:** Break into smaller functions:
- `processClockDetection()`
- `calculateDelayLength()`
- `processDrift()`
- `readDelayBuffer()`
- `processEffects()`
- `writeToBuffer()`

### 21. **Inconsistent Naming**
- Some variables use `pThis`, others use `alg`
- Mix of `L`/`R` and `L`/`R` suffixes
- Some abbreviations (`apd`, `lofi`) vs full words

**Recommendation:** Establish and follow a consistent naming convention.

### 22. **Missing Error Handling**
**Location:** Throughout

There's no error handling for:
- Invalid parameter values
- Buffer allocation failures (though API handles this)
- Division by zero (some protection exists, but not comprehensive)

**Recommendation:** Add bounds checking and validation where appropriate.

## Positive Aspects

✅ Good use of interpolation for smooth parameter changes
✅ Well-structured parameter organization with pages
✅ Comprehensive feature set (drift, diffusion, tape saturation, etc.)
✅ Good separation of concerns in helper functions
✅ Proper use of API patterns (parameterChanged, step, draw)
✅ Thoughtful crossfade implementation for loop transitions

## Recommendations Summary

### High Priority
1. Fix hardcoded 96kHz assumption - use `NT_globals.sampleRate`
2. Fix ping-pong logic bug (line 1005)
3. Remove or implement `freezeAnchor`
4. Fix buffer size documentation/clarity

### Medium Priority
5. Make clock detection threshold configurable or more robust
6. Break up long `step()` function
7. Add named constants for magic numbers
8. Cache filter coefficients

### Low Priority
9. Improve code organization and naming consistency
10. Add more comprehensive error handling
11. Optimize clock detection
12. Document smoothing rate choices

## Testing Recommendations

1. Test with different sample rates (48kHz, 96kHz, etc.)
2. Test ping-pong mode edge cases (exact boundaries)
3. Test clock detection with various signal levels
4. Test MIDI channel filtering (especially channel 0 = all)
5. Test freeze/unfreeze transitions
6. Test rapid parameter changes (division, loop mode)
7. Test with extreme parameter values
8. Test buffer wrap-around with very long delay times

