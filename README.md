# MrFreeze

A tempo-synced freeze delay for **Expert Sleepers disting NT** that loves to destroy your audio in the best way possible. Capture a moment, loop it, and watch it slowly (or quickly!) disintegrate into beautiful chaos. Each trip through the feedback loop mangles your sound with tape saturation, resonant filters, bit crushing, and diffusion—turning crisp loops into warbly, crunchy, otherworldly textures.

## ✨ Features

- **🎛 Tempo-Synced Delay**: Lock to internal BPM, external clock CV, or MIDI clock—your loops always stay in the pocket
- **🔄 Freeze/Loop Modes**: Forward, Reverse, and Ping-Pong playback with smooth crossfades
- **💥 Audio Degradation Chain** — the fun stuff:
  - Tape saturation with authentic hysteresis
  - Resonant HPF/LPF filters with tilt EQ for sculpting
  - Crunchy bit crushing and sample rate reduction
  - Lush all-pass diffusion network
  - Analog drift for that "vintage gear left on too long" vibe
- **📈 Envelope Generator**: Beat-synced attack/decay envelope for hands-free automation—perfect for build-ups, risers, and epic washouts
- **🎹 MIDI Keyboard Mapping**: Play your delay like an instrument! Intuitive note-to-division mapping with latch mode for live performance
- **🛡 Look-ahead Limiter**: Crank the feedback without fear—we've got you covered

## Parameter Pages

### Page 1: Routing & I/O
| Parameter | Description |
|-----------|-------------|
| Dry In L/R | Audio input selection |
| FB In L/R | External feedback input |
| Clock In | External clock CV input |
| Out L/R | Audio output selection |

### Page 2: Performance
| Parameter | Range | Description |
|-----------|-------|-------------|
| Freeze | Off/On | Enable loop capture |
| Division | 1/128 to 32/1 | Delay time division |
| Triplet | Off/×3/÷3 | Triplet modifier |
| Dotted | Off/×1.5 | Dotted note modifier |
| Crossfade | 0-50% | Loop crossfade amount |
| Ping Pong | Off/On | Stereo ping-pong |
| Loop Mode | Fwd/Rev/PingPong | Playback direction |
| Sync | Int/Clock/MIDI | Tempo sync source |
| Tempo | 30-300 BPM | Internal tempo |

### Page 3: Character
| Parameter | Range | Description |
|-----------|-------|-------------|
| Feedback | -∞ to +6dB | Feedback amount |
| Resonance | 0-100% | Filter resonance/Q |
| Base | 0-127 | High-pass filter cutoff |
| Width | 0-127 | Low-pass filter cutoff |
| Tone | 0-100% | Tilt EQ (dark ↔ bright) |
| Bit Depth | 1-24 | Bit crusher depth |
| Lo-Fi | 0-100% | Sample rate reduction |
| Tape Sat | 0-100% | Tape saturation amount |
| Diffusion | 0-100% | All-pass diffusion |
| Drift | 0-100% | Analog drift simulation |

### Page 4: Envelope Mod
| Parameter | Range | Description |
|-----------|-------|-------------|
| Env Atk | 1/2 beat to 32 bars | Attack time |
| Env Dec | 1/2 beat to 32 bars | Decay time |
| Env Curve | -100% to +100% | Log ↔ Lin ↔ Exp |
| Env>Base | -100% to +100% | Modulate HPF |
| Env>Width | -100% to +100% | Modulate LPF |
| Env>Tone | -100% to +100% | Modulate tilt EQ |
| Env>Res | -100% to +100% | Modulate resonance |
| Env>LoFi | -100% to +100% | Modulate lo-fi |
| Env>Diff | -100% to +100% | Modulate diffusion |

### Page 5: MIDI Mapping
| Parameter | Range | Description |
|-----------|-------|-------------|
| MIDI Ch | 0-15 | Channel filter (0=Omni) |
| MIDI Mode | Off/Momentary/Latch | Note behavior |
| Note Base | 1/128 to 1/2 | Division for C note |
| Black Keys | Dotted/Triplet | Black key modifier |

## MIDI Keyboard Mapping

The keyboard maps notes to time divisions for expressive performance:

```
│ C  │C#│ D  │D#│ E  │ F  │F#│ G  │G#│ A  │A#│ B  │
│ ×1 │×M│ ×2 │×M│ ×4 │ ×8 │×M│×16 │×M│×32 │×M│×64 │
└────┴──┴────┴──┴────┴────┴──┴────┴──┴────┴──┴────┘
```

- **White keys**: Power-of-2 divisions (each doubles the previous)
- **Black keys**: Modified version (dotted ×1.5 or triplet ×1.33) of preceding white key
- **Note Base**: Sets what division C plays (D is 2×, E is 4×, etc.)

### MIDI Modes

| Mode | Note On | Note Off |
|------|---------|----------|
| **Off** | Ignored | Ignored |
| **Momentary** | Freeze + set division | Unfreeze |
| **Latch** | Toggle freeze + set division | — |

In **Latch** mode, pressing the same note again unfreezes.

### Example (Note Base = 1/16, Black Keys = Dotted)

| Note | Division | Time @ 120 BPM |
|------|----------|----------------|
| C | 1/16 | 31ms |
| C# | 1/16. | 47ms |
| D | 1/8 | 63ms |
| D# | 1/8. | 94ms |
| E | 1/4 | 125ms |
| F | 1/2 | 250ms |
| F# | 1/2. | 375ms |
| G | 1/1 | 500ms |
| A | 2/1 | 1s |
| B | 4/1 | 2s |

## Envelope Generator

The envelope triggers when freeze is activated (via parameter, CV, or MIDI) and modulates character parameters over time.

### Trigger Behavior
- Triggers on freeze rising edge
- MIDI legato: no retrigger if notes are held
- Envelope completes its cycle even if freeze is released

### Curve Shapes
- **-100%**: Logarithmic (fast start, slow finish)
- **0%**: Linear
- **+100%**: Exponential (slow start, fast finish)

## Specifications

| Spec | Range | Default |
|------|-------|---------|
| Max Delay Time | 1-20 seconds | 10 seconds |

## Installation

1. Compile `MrFreezeNT.cpp` using the disting NT SDK
2. Copy the compiled `MrFreezeNT.o` to the `plugins` folder on your disting NT's SD card
3. Reboot the disting NT to load the algorithm

## Building

Requires the [Expert Sleepers disting NT SDK](https://expert-sleepers.co.uk/distingNT.html).

```bash
# Follow disting NT SDK build instructions
```

## License

MIT License - see [LICENSE](LICENSE) for details.

## Credits

Developed by **Nathan Harris** for the Expert Sleepers disting NT platform.

