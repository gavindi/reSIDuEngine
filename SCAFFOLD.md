# reSIDuEngine Architecture

This document describes the architectural design and data structures of the reSIDuEngine SID emulator.

## Architecture Overview

reSIDuEngine uses a **monolithic single-file architecture** where all SID emulation logic is contained within a single `SID` class:

- **Header**: `Source/reSIDuEngine.h` - Complete class interface, constants, and inline methods
- **Implementation**: `Source/reSIDuEngine.cpp` - Complete emulation implementation

This design prioritizes simplicity and performance by avoiding indirection layers, virtual functions, or PIMPL idioms.

## Core Components

The `SID` class emulates five major subsystems:

### 1. Voice Oscillators (3 voices)
Each voice contains:
- 24-bit phase accumulator (stored as `uint32_t`, masked `& 0xFFFFFF` each step)
- Fractional carry (`double`) for sub-integer accumulator advancement at audio-sample rate
- Frequency control (16-bit register)
- Waveform generators: Triangle, Sawtooth, Pulse, Noise
- 23-bit LFSR for noise generation
- Support for hard sync and ring modulation between voices

### 2. ADSR Envelope Generators (3 voices)
Each voice has an independent envelope with:
- State machine: Attack � Decay � Sustain � Release
- 8-bit envelope counter (0-255 amplitude)
- Rate counter for timing control
- Exponential decay implementation for logarithmic release curves
- 16 selectable rate values for each stage

### 3. Waveform Synthesis Engine
- Pure waveforms: Triangle, Sawtooth, Pulse (variable width), Noise
- Combined waveforms: Modeling analog interactions via lookup tables
- Band-limited synthesis to prevent aliasing
- Support for sync and ring modulation effects

### 4. Multi-Mode Filter
- State-variable (bi-quadratic) topology
- Simultaneous Lowpass, Bandpass, and Highpass outputs
- 11-bit cutoff frequency control mapped through a hardware-accurate R-2R ladder DAC lookup table
- 4-bit resonance control with per-model feedback formula
- Per-voice filter routing
- Separate filter characteristics for 6581 and 8580 chip models
- External RC output stage (LP ~16 kHz + HP ~1.6 Hz) and soft-clip post-processing

### 5. Signal Path and Mixing
- Per-voice filter routing (3 independent routing controls)
- Voice 3 output disable capability
- Master volume control (4-bit)
- External volume multiplier

## Data Structures

### Clock and Timing
```cpp
double sampleRate;          // Audio output rate (e.g., 44100 Hz)
double cpuClock;            // C64 CPU clock (985248 Hz PAL, 1022727 Hz NTSC)
double frameRate;           // Video frame rate (50 Hz for PAL, 60 for NTSC)
double clkRatio;            // CPU cycles per sample (cpuClock / sampleRate)
double frameSamplePeriod;   // Samples per frame (sampleRate / frameRate)
SIDModel sidModel;          // MOS6581 or MOS8580
float volume;               // Master volume multiplier
```

### Extended API State
```cpp
bool filterEnabled;                         // Filter enable/disable flag
std::array<bool, SID_CHANNELS> voiceMuted;  // Per-voice mute flags
int externalInput;                          // External audio input (EXT IN)
```

### Register Memory
```cpp
std::array<uint8_t, 32> SIDRegister;  // SID registers 0xD400-0xD41F
```

Register map (per voice, 7 bytes � 3):
- `+0x00`: Frequency low byte
- `+0x01`: Frequency high byte
- `+0x02`: Pulse width low byte
- `+0x03`: Pulse width high nibble
- `+0x04`: Control register (waveform, gate, sync, ring, test)
- `+0x05`: Attack/Decay rates
- `+0x06`: Sustain level/Release rate

Global registers:
- `0x15`: Filter cutoff low 3 bits
- `0x16`: Filter cutoff high byte
- `0x17`: Resonance (upper nibble) + filter routing (lower 3 bits)
- `0x18`: Filter modes (bits 4-6) + master volume (bits 0-3) + voice 3 off (bit 7)
- `0x1B`: Read-only: Voice 3 oscillator output
- `0x1C`: Read-only: Voice 3 envelope output

### ADSR Envelope State (per voice)
```cpp
std::array<uint8_t, SID_CHANNELS> adsrState;   // State flags
std::array<float, SID_CHANNELS> rateCounter;   // Rate counter
std::array<int, SID_CHANNELS> envelopeCounter; // Envelope level (0-255)
std::array<int, SID_CHANNELS> exponentCounter; // Exponential counter
std::array<uint8_t, SID_CHANNELS> previousSR;  // Previous sustain/release
```

State flags:
- `ATTACK_BITMASK (0x80)`: In attack phase
- `DECAYSUSTAIN_BITMASK (0x40)`: In decay/sustain phase
- `HOLDZERO_BITMASK (0x10)`: Holding at zero
- `GATE_BITMASK (0x01)`: Gate status

### Waveform Generation State (per voice)
```cpp
std::array<uint32_t, SID_CHANNELS> phaseAccumulator;      // Phase accumulator (24-bit integer)
std::array<double,   SID_CHANNELS> phaseAccumulatorFrac;  // Fractional carry for sub-sample advance
std::array<uint32_t, SID_CHANNELS> previousAccumulator;   // Previous phase (for edge detection)
std::array<uint32_t, SID_CHANNELS> noiseLFSR;             // 23-bit noise LFSR
std::array<uint16_t, SID_CHANNELS> previousWFOut;         // Previous waveform output
std::array<uint16_t, SID_CHANNELS> previousWaveData;      // Previous combined waveform

std::array<uint8_t,  SID_CHANNELS> sourceMSBrise; // MSB rising edge detect (for hard sync)
std::array<uint32_t, SID_CHANNELS> sourceMSB;     // Previous voice MSB (for ring modulation)
```

### DAC Nonlinearity Tables

All three SID DACs (oscillator, envelope, filter cutoff) are modelled as R-2R ladder networks
using `buildKinkedDacTable()`. Tables are precomputed at construction time.

```cpp
// Oscillator waveform DAC — 12-bit, 4096 entries per chip model.
// waveformOutput upper 12 bits (>> 4) index these tables.
// Values are centered at the DAC midpoint and scaled to ~[-32768, +32767].
std::array<float, 4096> oscDAC6581;  // 6581: kinked (2R/R=2.20) + cubic NMOS saturation
std::array<float, 4096> oscDAC8580;  // 8580: kinked (2R/R=2.00), no saturation

// Envelope DAC — 8-bit, 256 entries per chip model.
// envelopeCounter (0-255) indexes these tables; output normalized to [0, 1].
std::array<float, 256>  envDAC6581;  // 6581: kinked (2R/R=2.20, no bit-0 termination)
std::array<float, 256>  envDAC8580;  // 8580: kinked (2R/R=2.00, full termination)
```

### Filter State
```cpp
float previousLowpass;      // LP integrator state
float previousBandpass;     // BP integrator state

// Filter cutoff DAC — 11-bit, 2048 entries per chip model.
// FC register index → SVF cutoff coefficient, precomputed via R-2R DAC simulation.
std::array<float, 2048> fcCutoffTable6581;  // 6581: kinked DAC (2R/R=2.20, no bit-0 termination)
std::array<float, 2048> fcCutoffTable8580;  // 8580: linear DAC (2R/R=2.00, full termination)

// External RC output stage state and coefficients
float externalLowpass;   // LP filter state  (~16 kHz, 10kΩ/1000pF)
float externalHighpass;  // HP filter state  (~1.6 Hz, 10kΩ/10µF)
float w0lp;              // LP coefficient   (dt / (dt + RC))
float w0hp;              // HP coefficient   (dt / (dt + RC))
```

### Lookup Tables

#### Combined Waveform Tables (4096 entries each)
```cpp
std::array<uint16_t, 4096> triSaw8580;      // Triangle + Sawtooth
std::array<uint16_t, 4096> pulseSaw8580;    // Pulse + Sawtooth
std::array<uint16_t, 4096> pulseTriangle8580; // Pulse + Triangle
std::array<uint16_t, 4096> pulseTriSaw8580; // All three combined
```

These tables model analog waveform interactions when multiple waveform bits are set simultaneously. Generated during construction using `createCombinedWF()`.

The `pulseTriangle8580` table specifically models the pulse+triangle combination with parameters tuned for accurate SID behavior.

#### ADSR Timing Tables
```cpp
std::array<float, 16> adsrPeriods;    // Period for each rate (in CPU clock cycles, from LFSR simulation)
std::array<int, 256> adsrExptable;    // Exponential decay prescaler (hardware-measured thresholds)
```

## Voice Interconnections

Voices are interconnected in a circular topology for sync and ring modulation:

- **Voice 0**: Can sync to Voice 2, can be ring-modulated by Voice 2
- **Voice 1**: Can sync to Voice 0, can be ring-modulated by Voice 0
- **Voice 2**: Can sync to Voice 1, can be ring-modulated by Voice 1

This circular dependency is implemented using modulo arithmetic:
```cpp
int syncSource = (voiceIndex + 2) % SID_CHANNELS;  // Previous voice
```

## External Audio Input (EXT IN)

The SID chip has an external audio input that can be injected into the signal path:

- **Input method**: `input(int value)` - Accepts 16-bit signed samples
- **Routing**: Controlled by bit 3 (EXTIN_BITMASK) of register 0xD417
  - If EXTIN bit is set: routed through the filter
  - If EXTIN bit is clear: mixed directly with output
- **Consumption**: Each input sample is consumed after one `processSID()` call
- **Member variable**: `int externalInput` stores the current input sample

## Processing Pipeline

The `processSID()` method implements the main processing loop:

```
For each voice (0, 1, 2):

  1. ADSR Envelope Processing
     - Detect gate transitions (0->1 starts Attack, 1->0 starts Release)
     - Advance 15-bit LFSR rate counter; fire envelope step(s) when counter wraps
     - Update ADSR state machine (Attack / DecaySustain / HoldZero)
     - Apply exponential decay prescaler (adsrExptable) during Decay/Release

  2. Phase Accumulator Update
     - Accumulate fractional carry; add integer part to uint32_t accumulator
     - Mask to 24 bits (& 0xFFFFFF)
     - Apply hard sync: reset accumulator if sync source MSB rose
     - Detect MSB rising edge for sync source notification

  3. Waveform Generation
     - Select active waveform(s) from control register upper nibble
     - Apply ring modulation (XOR MSB of source voice) for triangle output
     - Generate waveform: triangle, sawtooth, pulse (with band-limiting), noise (LFSR)
     - Combined waveforms: table lookup via getMeasuredCombinedWF()

  4. Amplitude Modulation
     - Look up waveform analog voltage: `oscDAC[waveformOutput >> 4]`
       (12-bit index into chip-specific table; values centered at zero, scaled to ±32767)
     - Look up envelope analog voltage: `envDAC[envelopeCounter]`
       (8-bit index into chip-specific table; normalized [0, 1])
     - Multiply: `voiceOut = oscDAC[waveformOutput >> 4] * envDAC[envelopeCounter]`

  5. Signal Routing
     - Route voice output to filter input OR direct output based on RES_FILT bits
     - Check Voice 3 disable bit (OFF3_BITMASK) before adding to output

After all voices processed:

  6. Filter Processing
     - Decode 11-bit FC value: (FC_HI << 3) | (FC_LO & 7)
     - Look up cutoff coefficient from fcCutoffTable6581/8580[fc]
     - Compute resonance: 8580 uses pow(2,(4-res)/8); 6581 uses (~res & 0xf)/8.0
     - Update SVF integrators (LP, BP); mix enabled LP/BP/HP modes into output

  7. Master Volume & Output Scaling
     - Apply 4-bit master volume (bits 0-3 of 0xD418)
     - Scale by 1/0x10000 to normalize waveform amplitude

  8. External RC Output Stage
     - Lowpass at ~16 kHz (10 kOhm / 1000 pF): removes near-Nyquist content
     - Highpass at ~1.6 Hz  (10 kOhm / 10 uF):  removes DC drift
     - result = LP_state - HP_state

  9. Soft-Clip
     - If |result| > 1.0: apply tanh-based compression with headroom 0.5
     - Prevents harsh digital peaks at high voice counts or resonance settings
```

## Key Algorithms

### Phase Accumulator
The oscillator uses a 24-bit integer accumulator matching hardware exactly:

```cpp
double accumulatorAdd = (freqLo | (freqHi << 8)) * clkRatio;  // per-sample increment
phaseAccumulatorFrac[v] += accumulatorAdd;
uint32_t intAdd = static_cast<uint32_t>(phaseAccumulatorFrac[v]);
phaseAccumulatorFrac[v] -= intAdd;
phaseAccumulator[v] = (phaseAccumulator[v] + intAdd) & 0xFFFFFF;
```

The fractional carry preserves pitch accuracy when advancing once per audio sample rather than once per CPU cycle. MSB rising-edge detection for hard sync uses `(~previousAccumulator[v] & phaseAccumulator[v]) & 0x800000`.

### ADSR State Machine
The envelope uses a state machine with three main states:

1. **Attack**: Linear ramp from current level to 255
   - Triggered by gate 0�1 transition
   - Uses attack rate from register

2. **Decay/Sustain**: Exponential decay to sustain level, then hold
   - Entered when attack reaches 255
   - Uses decay rate, sustain level from registers

3. **Release**: Exponential decay to zero
   - Triggered by gate 1�0 transition
   - Uses release rate from register

Exponential behavior is achieved via the `adsrExptable[]` prescaler.

### Band-Limited Synthesis

#### Pulse Wave
At high frequencies, pulse edges are elongated to create trapezoidal transitions:
```cpp
double step = 256.0 / (accumulatorAdd >> 16);
waveformOutput = lim - (pulseWidth - tmp) * step;  // Linear interpolation
```

#### Sawtooth Wave
At high frequencies, the sawtooth becomes asymmetric to smooth the discontinuity:
```cpp
double step = accumulatorAdd / 0x1200000;
waveformOutput_d = waveformOutput + waveformOutput * step;
if (waveformOutput_d > 0xFFFF)
    waveformOutput_d = 0xFFFF - (waveformOutput_d - 0x10000) / step;
```

### Combined Waveforms
When multiple waveform bits are set, `createCombinedWF()` function models analog bit-fighting:

```cpp
for each output bit j:
    for each input bit k:
        influence = (bitmul / bitstrength^|k-j|) * (bit[k] - 0.5)
    if total_influence >= threshold:
        set output bit j
```

This simulates open-drain drivers affecting neighboring bits through resistive coupling.

Additionally, `getMeasuredCombinedWF()` function provides frequency-dependent filtering:
- Applies frequency-dependent coefficients (similar to analog RC circuit behavior)
- Uses previous sample feedback for capacitive coupling modeling
- Implements chip-specific differences between 6581 and 8580

### Noise Generator
Noise uses a 23-bit LFSR with taps at bits 22 and 17:

```cpp
step = (lfsr & 0x400000) ^ ((lfsr & 0x20000) << 5);
lfsr = ((lfsr << 1) + (step > 0)) & 0x7FFFFF;
```

Output is extracted from 8 specific LFSR bits for good spectral characteristics.

### State-Variable Filter
Two-integrator loop with simultaneous LP/BP/HP outputs:

```cpp
// Cutoff: table lookup (precomputed via R-2R DAC simulation)
const int fc = (SIDRegister[0x16] << 3) | (SIDRegister[0x15] & 0x07);
double cutoff = (sidModel == MOS8580) ? fcCutoffTable8580[fc] : fcCutoffTable6581[fc];

// Resonance feedback (1/Q):
//   8580: pow(2, (4 - res) / 8)          range ~0.38-1.41
//   6581: (~res & 0xf) / 8.0, clamped >= 0.3   range 0.30-1.875

double tmp = filterInput + previousBandpass * resonance + previousLowpass;  // HP
previousBandpass = previousBandpass - tmp * cutoff;                          // BP
previousLowpass  = previousLowpass  + previousBandpass * cutoff;             // LP
// Mix enabled modes into output, scaled by resonanceCompensation
```

After mixing, the output passes through the external RC stage and soft-clip (see Processing Pipeline steps 8–9).

## Chip Model Differences

### MOS 6581 (Original)
- **Oscillator DAC**: R-2R with 2R/R=2.20 and missing bit-0 termination; lower bits have compressed, non-monotonic weights. Output additionally passes through a cubic NMOS source-follower saturation model (`1.1V − 0.121V³`), adding harmonic distortion at higher amplitudes.
- **Envelope DAC**: R-2R with 2R/R=2.20 and missing bit-0 termination; envelope scaling is non-linear (lower levels decay faster than ideal).
- **Filter cutoff**: Same kinked R-2R (2R/R=2.20, no bit-0 termination). Compresses the low FC range, producing a natural dead zone. Table is monotonized post-construction (the raw DAC is non-monotonic at FC_LO roll-overs). Max coefficient calibrated to 16 kHz.
- **Resonance**: `(~res & 0xf) / 8.0` (hardware-measured), clamped at 0.3 (Q ≈ 3.3 max) to prevent SVF instability.
- **Combined waveforms**: 11-bit output (MSB masked)

### MOS 8580 (Revised)
- **Oscillator DAC**: R-2R with 2R/R=2.00 and full termination; near-monotonic response. No NMOS saturation modelled.
- **Envelope DAC**: R-2R with 2R/R=2.00 and full termination; near-linear scaling.
- **Filter cutoff**: R-2R with 2R/R=2.00 and full termination. Produces a near-linear response. Max coefficient calibrated to 12.5 kHz (matches old formula to within 0.5%).
- **Resonance**: `pow(2, (4 - res) / 8)`, Q range 0.71–2.59.
- **Combined waveforms**: Full 12-bit output

## Memory Layout

The SID uses 32 bytes of register space organized as follows:

```
0x00-0x06: Voice 0 (7 bytes)
0x07-0x0D: Voice 1 (7 bytes)
0x0E-0x14: Voice 2 (7 bytes)
0x15-0x18: Filter control (4 bytes)
0x19-0x1A: Padding/unused (write-only)
0x1B:      Voice 3 oscillator (read-only)
0x1C:      Voice 3 envelope (read-only)
0x1D-0x1F: Padding/unused
```

## Performance Characteristics

- **Sample Rate**: Configurable (typically 44100 or 48000 Hz)
- **Processing**: Single-threaded, per-sample processing
- **Memory**: Fixed allocation, no dynamic memory during operation
- **Precision**: Double-precision phase accumulators, single-precision audio output
- **Latency**: Zero-latency (real-time sample generation)

## Thread Safety

The `SID` class is **not thread-safe**. External synchronization is required if:
- Multiple threads call `write()`, `read()`, or `clock()`
- One thread writes registers while another calls `clock()`

Typical usage is single-threaded within an audio callback.

## Extended API

### Chip Model Aliases
- `setChipModel(SIDModel)` / `getChipModel()` - Aliases for `setModel()` / `getModel()`
- Both naming conventions are supported simultaneously

### Additional Methods
- `input(int value)` - External audio input injection (EXT IN)
- `enableFilter(bool)` - Enable/disable filter emulation
- `mute(int channel, bool)` - Per-voice muting
- `setSamplingParameters(clockFreq, method, samplingFreq, highestAccurateFreq)` - Clock and sample rate configuration; rebuilds cutoff tables and RC coefficients
- `clockSilent(unsigned int cycles)` - Advance state without audio generation

### Static Helper Functions (file scope, not class members)
- `computeLFSRPeriod(target)` - Simulates the 15-bit LFSR to count clocks from 0x7fff to a target value; used to build `adsrPeriods[]`
- `buildKinkedDacTable(out, bits, rRatio, hasTerm, leakage)` - Builds a normalized R-2R ladder DAC lookup table for any bit width; used to populate all six chip-specific DAC tables: `oscDAC6581/8580` (12-bit), `envDAC6581/8580` (8-bit), and `fcCutoffTable6581/8580` (11-bit)

### Combined Waveform Functions
- `createCombinedWF(waveformArray, bitmul, bitstrength, threshold)` - Generate lookup tables modeling analog bit interactions
- `combinedWF(voiceIndex, waveformArray, index, differ6581)` - Look up and interpolate combined waveform values
- `getMeasuredCombinedWF(voiceIndex, index, waveformCtrl)` - Advanced frequency-dependent filtering with feedback

### Constants
- `C64_PAL_CPUCLK = 985248.0` - PAL C64 CPU clock frequency (Hz)
- `PAL_FRAMERATE = 50.0` - PAL video frame rate (Hz)
- `C64_NTSC_CPUCLK = 1022727.0` - NTSC C64 CPU clock frequency (Hz)
- `NTSC_FRAMERATE = 60.0` - NTSC video frame rate (Hz, rounded from 59.826)
- `EXTIN_BITMASK = 0x08` - External input filter routing bit (register 0xD417)
