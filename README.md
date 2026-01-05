# reSIDuEngine

A C++ implementation of the SID 6581/8580

## Overview

This project provides high-quality emulation of the MOS6581 and MOS8580 SID (Sound Interface Device) chips used in the Commodore 64.

The implementation focuses on:
- **Accurate ADSR envelope generation** with proper attack/decay/sustain/release phases
- **Band-limited waveform synthesis** to reduce aliasing at high frequencies
- **Combined waveform support** modeling the analog behavior of the SID chip
- **Bi-quadratic filter** implementation for lowpass, bandpass, and highpass modes
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
- Exponential decay curves
- Proper rate counters and prescalers
- ADSR delay bug emulation for authentic sound
- Hold-zero state handling

### Filter
Two-integrator loop bi-quadratic filter supporting:
- Lowpass, bandpass, and highpass modes
- Resonance control
- Per-voice filter routing
- External audio input (EXT IN) routing
- Separate 6581 and 8580 cutoff curves

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

### ADSR Periods (in clock cycles)

| Rate | Cycles | Step |
|------|--------|------|
| 0    | 9      | ceil(9/9) |
| 1    | 32     | 1 |
| 2    | 63     | 1 |
| 3    | 95     | 1 |
| 4    | 149    | 1 |
| 5    | 220    | 1 |
| 6    | 267    | 1 |
| 7    | 313    | 1 |
| 8    | 392    | 1 |
| 9    | 977    | 1 |
| 10   | 1954   | 1 |
| 11   | 3126   | 1 |
| 12   | 3907   | 1 |
| 13   | 11720  | 1 |
| 14   | 19532  | 1 |
| 15   | 31251  | 1 |

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
- Warmer, darker filter sound
- Non-linear filter cutoff with dead zone
- Combined waveforms use 11-bit output (MSB masked)

**MOS8580:**
- Brighter, cleaner filter sound
- More linear filter cutoff (no dead zone)
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
