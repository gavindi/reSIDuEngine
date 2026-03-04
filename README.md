# reSIDuEngine

![reSIDuEngine](assets/logo/reSIDuEngine1.png)

A C++ implementation of the SID 6581/8580

## Overview

This project provides high-quality emulation of the MOS6581 and MOS8580 SID (Sound Interface Device) chips used in the Commodore 64.

The implementation focuses on:
- **Accurate ADSR envelope generation** with proper attack/decay/sustain/release phases
- **Band-limited waveform synthesis** to reduce aliasing at high frequencies
- **Combined waveform support** modeling the analog behavior of the SID chip
- **Bi-quadratic filter** implementation for lowpass, bandpass, and highpass modes
- **Hardware-accurate DAC nonlinearity** for oscillator, envelope, and filter cutoff paths
- **Noise generator** using a 23-bit LFSR
- **Sync and ring modulation** between voices
- **reSIDfp API compatibility** for easy migration from reSIDfp
- **External audio input (EXT IN)** support with filter routing

## Key Features

### Waveform Generation
- **Triangle** - Clean triangle waves with optional ring modulation
- **Sawtooth** - Band-limited sawtooth with anti-aliasing for high pitches
- **Pulse** - Variable pulse width with band-limiting to prevent aliasing
- **Noise** - 23-bit LFSR-based white noise generator
- **Combined waveforms** - Accurate modeling of analog interactions between waveforms

### Combined Waveforms
The implementation includes sophisticated modeling of how the SID's analog circuitry produces combined waveforms when multiple waveform bits are enabled simultaneously. This uses recursive calculations to simulate how neighboring bits affect each other through the chip's open-drain drivers and FET switches.

### ADSR Envelope Generator
Accurate envelope generation with:
- Hardware-accurate 15-bit LFSR rate counter timing (periods derived by LFSR simulation, not approximation)
- Exponential decay curves with hardware-measured threshold values (`0x5d`, `0x36`, `0x1a`, `0x0e`, `0x06`)
- ADSR delay bug emulation for authentic sound
- Hold-zero state handling

### DAC Nonlinearity
All three SID DAC paths are modelled as chip-specific R-2R ladder networks (ported from the residfpII circuit model), replacing the ideal linear conversion used in simpler emulators:
- **Oscillator DAC** (12-bit): chip-specific R-2R nonlinearity; MOS6581 additionally applies a cubic NMOS source-follower saturation model, adding harmonic distortion at higher amplitudes
- **Envelope DAC** (8-bit): chip-specific R-2R nonlinearity; lower envelope levels decay with a slightly different shape than ideal
- **Filter cutoff DAC** (11-bit): chip-specific R-2R nonlinearity mapped to the SVF coefficient

### Filter
Two-integrator loop state-variable filter supporting:
- Lowpass, bandpass, and highpass modes (combinable)
- Hardware-accurate cutoff mapping via R-2R ladder DAC simulation (per chip model)
- Hardware-measured resonance feedback formula per chip model
- Per-voice filter routing
- External audio input (EXT IN) routing
- External RC output stage: ~16 kHz lowpass + ~1.6 Hz highpass for DC removal
- Soft-clip on final output to prevent harsh digital peaks at high resonance or volume

### Anti-Aliasing
Special band-limiting techniques for pulse and sawtooth waveforms:
- Frequency-dependent edge elongation for pulse waves
- Asymmetric triangle approximation for high-frequency sawtooth
- Maintains crisp sound at low frequencies while preventing aliasing at high frequencies

## Building

### Requirements
- CMake 3.10 or higher
- C++17 compatible compiler

### Build Instructions

```bash
mkdir build
cd build
cmake ..
make
```

This will build:
- `libresiduengine.a` - The reSIDuEngine static library
- `reSIDuEngine_example1` - Basic example (generates WAV file)
- `simple_test` - Simple test program (generates raw audio)
- `sid_player` - SID file player with SDL2 (if SDL2 available)
- `sid_player_portaudio` - SID file player with PortAudio (if PortAudio available)
- `sid_player_miniaudio` - SID file player with miniaudio (if miniaudio.h available)

### Running the Examples

```bash
# Example 1: Generates reSIDuEngine_output.wav (C major chord demo)
./build/examples/reSIDuEngine_example1

# Simple test: Generates output.raw
./build/examples/simple_test

# Play raw audio (requires aplay)
aplay -f S16_LE -r 44100 -c 1 output.raw

# SID Player: Play .sid files (Commodore 64 music format)
./build/examples/sid_player song.sid [subtune] [chip_model] [seconds]
./build/examples/sid_player_portaudio song.sid 1 8580
./build/examples/sid_player_miniaudio song.sid

# Download SID files from HVSC (High Voltage SID Collection):
# https://www.hvsc.c64.org/
```

See `examples/AUDIO_BACKENDS.md` for detailed comparison of the different SID player implementations.

## Usage

### Basic Example

```cpp
#include "reSIDuEngine.h"

// Create SID engine (44.1kHz sample rate, 8580 model)
reSIDuEngine::SID sid(44100, reSIDuEngine::MOS8580);

// Set voice 1 to sawtooth at 440 Hz (A4)
// Calculate frequency register: (note_freq * 16777216) / clock_freq
uint16_t freq = (440.0 * 16777216.0) / 985248.0;

// Write frequency
sid.write(0xD400, freq & 0xFF);        // FREQ_LO
sid.write(0xD401, (freq >> 8) & 0xFF); // FREQ_HI

// Set ADSR: Attack=8, Decay=6, Sustain=0, Release=6
sid.write(0xD405, 0x86); // ATTACK_DECAY
sid.write(0xD406, 0x06); // SUSTAIN_RELEASE

// Enable sawtooth with gate on
sid.write(0xD404, reSIDuEngine::SAW_BITMASK | reSIDuEngine::GATE_BITMASK);

// Set volume
sid.write(0xD418, 0x0F); // Volume 15

// Generate audio samples
for (int i = 0; i < numSamples; i++) {
    float sample = sid.clockSample();
    // Use sample...
}
```

### Register Map

Each voice has 7 registers (Voice 1: 0xD400-0xD406, Voice 2: 0xD407-0xD40D, Voice 3: 0xD40E-0xD414):

- `+0` FREQ_LO - Frequency low byte
- `+1` FREQ_HI - Frequency high byte
- `+2` PW_LO - Pulse width low byte
- `+3` PW_HI - Pulse width high byte (bits 0-3)
- `+4` CONTROL - Waveform and gate control
- `+5` ATTACK_DECAY - Attack (high nibble) and Decay (low nibble)
- `+6` SUSTAIN_RELEASE - Sustain (high nibble) and Release (low nibble)

Global registers:
- `0xD415` FC_LO - Filter cutoff low bits
- `0xD416` FC_HI - Filter cutoff high byte
- `0xD417` RES_FILT - Resonance and filter routing
- `0xD418` MODE_VOL - Filter mode and master volume

### Control Register Bits

```cpp
GATE_BITMASK     = 0x01  // Gate on/off
SYNC_BITMASK     = 0x02  // Sync to previous voice
RING_BITMASK     = 0x04  // Ring modulation
TEST_BITMASK     = 0x08  // Test bit (holds oscillator)
TRI_BITMASK      = 0x10  // Triangle waveform
SAW_BITMASK      = 0x20  // Sawtooth waveform
PULSE_BITMASK    = 0x40  // Pulse waveform
NOISE_BITMASK    = 0x80  // Noise waveform
```

### Filter Mode Bits (0xD418)

```cpp
LOWPASS_BITMASK  = 0x10  // Lowpass filter
BANDPASS_BITMASK = 0x20  // Bandpass filter
HIGHPASS_BITMASK = 0x40  // Highpass filter
OFF3_BITMASK     = 0x80  // Turn off voice 3
```

### Filter Routing Bits (0xD417)

```cpp
EXTIN_BITMASK    = 0x08  // Route external audio input to filter
```

## reSIDfp API Compatibility

reSIDuEngine provides API compatibility with reSIDfp for easy migration:

### Core Methods (Compatible)
- `write(addr, value)` - Write to SID registers
- `read(addr)` - Read from SID registers
- `clock(cycles, buf)` - Generate audio samples
- `clockSilent(cycles)` - Advance state without audio output
- `reset()` - Reset to power-on state
- `setSamplingParameters()` - Configure clock and sample rate
- `enableFilter(enable)` - Enable/disable filter emulation
- `mute(channel, enable)` - Mute individual voices
- `input(value)` - External audio input (EXT IN)

### Chip Model Methods
- `setChipModel(model)` / `getChipModel()` - reSIDfp naming
- `setModel(model)` / `getModel()` - reSIDuEngine naming

Both naming conventions are supported for maximum compatibility.

### Additional reSIDuEngine Methods
- `clockSample()` - Generate single audio sample (convenience method)
- `setVolume(float)` - Set master volume multiplier
- `setClock(double)` - Set CPU clock frequency
- `setFramerate(double)` - Set frame rate for frame-synchronized timing

### Migration Example

Migrating from reSIDfp to reSIDuEngine is straightforward:

```cpp
// reSIDfp code
reSIDfp::SID sid;
sid.setChipModel(reSIDfp::MOS8580);
sid.setSamplingParameters(985248, reSIDfp::RESAMPLE_INTERPOLATE, 44100);

// Equivalent reSIDuEngine code (constructor approach)
reSIDuEngine::SID sid(44100, reSIDuEngine::MOS8580);
sid.setClock(985248);

// OR using reSIDfp-compatible API
reSIDuEngine::SID sid(44100);
sid.setChipModel(reSIDuEngine::MOS8580);
sid.setSamplingParameters(985248, 0, 44100);
```

## Advanced Usage

### External Audio Input (EXT IN)

The SID chip has an external audio input that can mix external audio with SID output:

```cpp
// Inject external audio sample (16-bit signed)
sid.input(externalSample);

// Route external input through filter (set bit 3 of 0xD417)
sid.write(0xD417, 0x08);  // Route EXT IN to filter

// Or mix directly with output (clear bit 3)
sid.write(0xD417, 0x00);  // EXT IN bypasses filter
```

### Voice Muting

Individual voices can be muted for debugging or selective playback:

```cpp
// Mute voice 2 (0-indexed)
sid.mute(1, true);

// Unmute voice 2
sid.mute(1, false);
```

### Filter Control

```cpp
// Disable filter emulation (all voices bypass filter)
sid.enableFilter(false);

// Re-enable filter
sid.enableFilter(true);
```

### Using clock() for Cycle-Accurate Emulation

For cycle-accurate emulation (e.g., with CPU emulation):

```cpp
// Generate samples based on CPU cycles elapsed
unsigned int cpuCycles = 1000;
short buffer[128];
int numSamples = sid.clock(cpuCycles, buffer);

// Process numSamples from buffer...
```

## Implementation Notes

### DAC Nonlinearity Implementation

All three SID DAC paths use the same `buildKinkedDacTable()` function (ported from the residfpII `Dac::kinkedDac()` circuit model), which simulates the R-2R resistor ladder by repeated parallel substitution. The key chip-specific parameters are:

| Parameter | MOS6581 | MOS8580 |
|---|---|---|
| 2R/R ratio | 2.20 | 2.00 |
| Bit-0 termination | Missing | Present |
| MOSFET leakage | 0.0075 | 0.0035 |
| NMOS saturation | Yes (cubic) | No |

**Oscillator DAC** (12-bit, 4096 entries): The waveform output's upper 12 bits index a precomputed table. Values are centered at the DAC midpoint and scaled to ±32767. For MOS6581, a cubic saturation curve (`V' = 1.1V − 0.121V³`) is applied after the R-2R calculation, modelling the NMOS source-follower output stage.

**Envelope DAC** (8-bit, 256 entries): The 8-bit envelope counter indexes a precomputed table normalized to [0, 1]. The nonlinear bit weights mean lower envelope levels have a slightly compressed response compared to ideal.

**Filter cutoff DAC** (11-bit, 2048 entries): The 11-bit FC register indexes a precomputed table of SVF cutoff coefficients. For MOS6581, the missing bit-0 termination compresses the low FC range; the table is monotonized post-construction since real hardware smooths the raw non-monotonicity analogically. The MOS8580 table produces a near-linear response.

After the SVF, the output passes through a two-stage RC model of the C64's audio output stage:
- **Lowpass ~16 kHz** (10 kΩ / 1000 pF): transparent at audio rates, removes near-Nyquist content
- **Highpass ~1.6 Hz** (10 kΩ / 10 µF): removes DC drift from the filter integrators
- **Soft-clip**: `tanh`-based compression above ±1.0 (headroom 0.5) prevents harsh digital peaks

### Oscillator Phase Accumulator

The phase accumulator is a 24-bit integer (`uint32_t`) with a `& 0xFFFFFF` mask on each
update, matching the hardware's integer counter exactly. A separate `double` fractional carry
preserves pitch accuracy when advancing the accumulator once per audio sample (rather than
once per CPU cycle). MSB rising-edge detection for hard sync uses the XOR method
`(~old & new) & 0x800000`, which correctly identifies the 0→1 transition of bit 23.

### Combined Waveforms

The combined waveform generation uses a novel approach discovered through analyzing the SID's internal circuitry:

1. The R-2R ladder WAVE DAC is driven by FET output drivers that switch based on voltage thresholds
2. When multiple waveforms are selected, their open-drain drivers are connected through wave-selector FETs
3. These drivers load each other, pulling analog levels up/down based on distance and state
4. The resulting analog level either exceeds the FET gate threshold or not, creating the combined waveform

This is implemented via the `createCombinedWF()` function which generates lookup tables modeling this behavior.

### Anti-Aliasing Approach

Traditional oversampling was too CPU-intensive, so a frequency-domain approach is used:
- Pulse waves: Edge transitions are elongated in a frequency-dependent way, making them trapezoidal at high frequencies
- Sawtooth waves: Become asymmetric triangles at high frequencies while staying sharp at low frequencies
- This matches the behavior seen in real SID recordings

## Technical Details

### ADSR Periods (in CPU clock cycles)

Periods are computed by simulating the hardware 15-bit LFSR and counting clocks
from the reset value (`0x7fff`) to the rate-specific comparison value.

| Rate | Cycles | Approx. time (full 0→255 attack at PAL) |
|------|--------|------------------------------------------|
| 0    | 8      | ~2 ms   |
| 1    | 31     | ~8 ms   |
| 2    | 62     | ~16 ms  |
| 3    | 94     | ~24 ms  |
| 4    | 148    | ~38 ms  |
| 5    | 219    | ~56 ms  |
| 6    | 266    | ~68 ms  |
| 7    | 312    | ~80 ms  |
| 8    | 391    | ~100 ms |
| 9    | 976    | ~250 ms |
| 10   | 1953   | ~500 ms |
| 11   | 3125   | ~800 ms |
| 12   | 3906   | ~1 s    |
| 13   | 11719  | ~3 s    |
| 14   | 19531  | ~5 s    |
| 15   | 31250  | ~8 s    |

### Frequency Calculation

The SID frequency register is calculated as:

```
freq_register = (desired_frequency_Hz * 16777216) / clock_frequency_Hz
```

For PAL systems: clock = 985248 Hz
For NTSC systems: clock = 1022727 Hz

### Chip Model Selection

Both MOS6581 (original) and MOS8580 (revised) chip models are supported:

**MOS6581:**
- Warmer, darker sound with harmonic distortion from NMOS source-follower saturation on the oscillator output
- Oscillator DAC: kinked R-2R (2R/R=2.20, missing bit-0 termination) + cubic saturation
- Envelope DAC: kinked R-2R (2R/R=2.20), non-linear amplitude scaling
- Filter cutoff DAC: same kinked R-2R; compresses the low end of the FC range, creating a natural dead zone
- Resonance feedback = `(~res & 0xf) / 8.0` (hardware-measured), clamped at Q≈3.3 maximum
- Combined waveforms use 11-bit output (MSB masked)

**MOS8580:**
- Brighter, cleaner sound; no oscillator saturation
- Oscillator DAC: near-ideal R-2R (2R/R=2.00, proper termination)
- Envelope DAC: near-ideal R-2R (2R/R=2.00), near-linear amplitude scaling
- Filter cutoff DAC: near-ideal R-2R, full FC range usable
- Resonance feedback = `pow(2, (4 - res) / 8)`, Q range 0.71–2.59
- Combined waveforms use full 12-bit output

Select chip model in constructor or use `setChipModel()` / `setModel()`.

## Documentation

- **README.md** - This file (user guide and API reference)
- **BUILD.md** - Build instructions and example commands
- **SCAFFOLD.md** - Detailed architectural documentation
- **CHANGELOG.md** - Version history and changes
- **CLAUDE.md** - Developer guide for working with the codebase
- **examples/AUDIO_BACKENDS.md** - SID player audio backend comparison

## License

reSIDuEngine is licensed under the Mozilla Public License Version 2.0 (MPL-2.0).
