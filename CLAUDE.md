# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

reSIDuEngine is a C++ implementation of the MOS 6581/8580 SID (Sound Interface Device) chip used in the Commodore 64. The project provides accurate emulation of the SID's waveform synthesis, ADSR envelopes, filters, and combined waveform behavior.

## Build System

This project uses CMake (minimum version 3.10) with C++17.

### Building from Scratch

```bash
mkdir -p build
cd build
cmake ..
cmake --build .
```

Or from project root:
```bash
cmake --build build
```

### Clean Build

```bash
rm -rf build
mkdir build
cd build
cmake ..
cmake --build .
```

### Build Outputs

- `build/libresiduengine.a` - Static library
- `build/examples/reSIDuEngine_example1` - Example program (generates WAV file)
- `build/examples/simple_test` - Simple test program
- `build/examples/sid_player` - SID file player with SDL2 (if SDL2 found)
- `build/examples/sid_player_portaudio` - SID file player with PortAudio (if PortAudio found)
- `build/examples/sid_player_miniaudio` - SID file player with miniaudio (if miniaudio.h found)

### Running Examples

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

## Architecture Overview

The codebase uses a single-file implementation architecture where all components are contained in:
- `Source/reSIDuEngine.h` - Complete SID emulator class interface
- `Source/reSIDuEngine.cpp` - Complete implementation

The implementation is monolithic and self-contained. See SCAFFOLD.md for detailed architectural documentation.

### Core Components

The `SID` class contains:

1. **Three Voice Channels**: Each voice has independent oscillators with waveform generation (triangle, sawtooth, pulse, noise), ADSR envelope generators, and support for sync and ring modulation between voices.

2. **Waveform Generation**:
   - 24-bit phase accumulators for frequency accuracy
   - Band-limited synthesis for pulse and sawtooth to reduce aliasing
   - 23-bit LFSR for noise generation
   - Combined waveform support (when multiple waveform bits are set)

3. **ADSR Envelopes**: Accurate envelope timing with exponential decay curves and proper rate counters.

4. **Multi-Mode Filter**: Bi-quadratic state-variable filter supporting lowpass, bandpass, and highpass modes with per-voice routing and resonance control. Separate implementations for 6581 and 8580 filter characteristics.

5. **Clock and Sample Rate**: The SID runs at PAL clock frequency (985248 Hz) and generates output at the configured sample rate (typically 44100 Hz).

### Voice Interconnections

- Voice 1 can sync/ring-modulate with Voice 3
- Voice 2 can sync/ring-modulate with Voice 1
- Voice 3 can sync/ring-modulate with Voice 2

This circular dependency is important for understanding voice interactions.

## Register Map

The SID uses memory-mapped I/O with registers at addresses 0xD400-0xD418:

**Per-Voice Registers** (7 bytes × 3 voices):
- `+0x00/0x07/0x0E`: FREQ_LO (frequency low byte)
- `+0x01/0x08/0x0F`: FREQ_HI (frequency high byte)
- `+0x02/0x09/0x10`: PW_LO (pulse width low byte)
- `+0x03/0x0A/0x11`: PW_HI (pulse width high 4 bits)
- `+0x04/0x0B/0x12`: CONTROL (waveform, gate, sync, ring, test)
- `+0x05/0x0C/0x13`: ATTACK_DECAY
- `+0x06/0x0D/0x14`: SUSTAIN_RELEASE

**Global Registers**:
- `0xD415`: FC_LO (filter cutoff low 3 bits)
- `0xD416`: FC_HI (filter cutoff high byte)
- `0xD417`: RES_FILT (resonance + voice filter routing)
- `0xD418`: MODE_VOL (filter modes + master volume)
- `0xD41B`: OSC3 (read-only: voice 3 oscillator output)
- `0xD41C`: ENV3 (read-only: voice 3 envelope output)

## Frequency Calculation

Frequency register value is calculated as:
```
freq_register = (note_frequency_Hz * 16777216) / clock_frequency_Hz
```

Where:
- PAL clock: 985248 Hz (C64_PAL_CPUCLK constant)
- NTSC clock: 1022727 Hz
- 16777216 = 2^24 (phase accumulator size)

## Key Implementation Details

### Combined Waveforms

When multiple waveform bits are enabled simultaneously, the SID produces combined waveforms through analog interaction of the chip's internal circuitry. The implementation models this using lookup tables that simulate how neighboring bits affect each other through open-drain drivers and FET switches.

### Anti-Aliasing

The implementation uses frequency-dependent techniques:
- **Pulse waves**: Edge transitions are elongated at high frequencies (trapezoidal)
- **Sawtooth waves**: Become asymmetric triangles at high frequencies
- This avoids CPU-intensive oversampling while preventing aliasing artifacts

### Chip Model Differences

**MOS6581** (original):
- Darker, warmer filter sound
- Non-linear "kinked" DAC
- More complex combined waveform behavior

**MOS8580** (revised):
- Brighter, cleaner filter sound
- More linear DAC
- Simpler combined waveform behavior

## Constants and Bitmasks

Important constants defined in the header:
- `SID_CHANNELS = 3`
- `C64_PAL_CPUCLK = 985248.0`
- `PAL_FRAMERATE = 50.0`

Control register bits:
- `GATE_BITMASK = 0x01` (start/stop envelope)
- `SYNC_BITMASK = 0x02` (hard sync)
- `RING_BITMASK = 0x04` (ring modulation)
- `TEST_BITMASK = 0x08` (halt oscillator)
- `TRI_BITMASK = 0x10` (triangle wave)
- `SAW_BITMASK = 0x20` (sawtooth wave)
- `PULSE_BITMASK = 0x40` (pulse wave)
- `NOISE_BITMASK = 0x80` (noise)

Filter mode bits (0xD418):
- `LOWPASS_BITMASK = 0x10`
- `BANDPASS_BITMASK = 0x20`
- `HIGHPASS_BITMASK = 0x40`
- `OFF3_BITMASK = 0x80` (disable voice 3 output)

Filter routing bits (0xD417):
- `EXTIN_BITMASK = 0x08` (route external audio input to filter)

## Examples

### Basic Examples

- **reSIDuEngine_example1.cpp** - Demonstrates basic usage by generating a C major chord and saving to WAV file
- **simple_test.cpp** - Minimal example showing how to initialize and use the SID emulator

### SID Player Examples

Three complete SID file player implementations demonstrating integration with CPU emulation and real-time audio output:

- **sid_player.cpp** (SDL2) - Uses SDL2 for cross-platform audio output
- **sid_player_portaudio.cpp** (PortAudio) - Uses PortAudio for professional-grade cross-platform audio
- **sid_player_miniaudio.cpp** (miniaudio) - Uses single-header miniaudio library (easiest to integrate)

All three players:
- Load and parse .sid files (Commodore 64 music format)
- Emulate 6502 CPU to run the music player code
- Support up to 3 SID chips for multi-SID tunes
- Support both MOS6581 and MOS8580 chip models
- Handle subtune selection

**Key implementation details:**
- Register writes from CPU are forwarded to reSIDuEngine via `write()`
- Audio samples generated via `clock()` once per output sample
- Output scaled by 0.25 to prevent clipping
- Initial register sync after CPU initialization to capture init-time writes

See `examples/AUDIO_BACKENDS.md` for detailed comparison and setup instructions.

### Audio Backend Installation

**SDL2:**
```bash
# Debian/Ubuntu: sudo apt-get install libsdl2-dev
# Fedora: sudo dnf install SDL2-devel
# macOS: brew install sdl2
```

**PortAudio:**
```bash
# Debian/Ubuntu: sudo apt-get install portaudio19-dev
# Fedora: sudo dnf install portaudio-devel
# macOS: brew install portaudio
```

**miniaudio:**
```bash
# Download single header file
cd examples/
wget https://raw.githubusercontent.com/mackron/miniaudio/master/miniaudio.h
```

## Code Style and Conventions

### Variable Naming

The codebase uses descriptive variable names for clarity:

- `voiceIndex` - Loop index for iterating over voices (0-2)
- `accumulatorAdd` - Frequency increment added to phase accumulator each sample
- `pulseWidth` - Pulse waveform width parameter (12-bit value, shifted left 4)
- `waveform` - Current waveform selection bits (upper nibble of control register)
- `SIDRegister` - Array holding all 32 SID register values (0xD400-0xD41F)
- `voiceRegister` - Pointer to current voice's 7-byte register block

These names replace earlier short names (voice→voiceIndex, accuadd→accumulatorAdd, etc.) to improve code readability.

## Documentation Files

- `README.md` - User-facing documentation with usage examples
- `BUILD.md` - Build instructions and example commands
- `SCAFFOLD.md` - Detailed architectural documentation describing the monolithic single-file design
- `CHANGELOG.md` - Version history and changes
- `examples/AUDIO_BACKENDS.md` - Comparison guide for SID player audio backends
