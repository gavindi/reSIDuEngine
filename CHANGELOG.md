# Changelog

All notable changes to reSIDuEngine will be documented in this file.

## [1.9.4] - 2026-07-08

### Fixed
- `sidplayer2`: Loader no longer drops the final byte of the SID file. The read loop left `datalen` at the exact file size but a stray `datalen--` combined with the `i < datalen` load bound truncated the last data byte. This silently corrupted tunes whose data ends exactly at a used address — e.g. Martin Galway's `Wizball`, whose last byte is the high byte of a raster-split jump-table entry; without it the split jumped to `$0003` and hung the raster-split counter, so the music tick never fired (silent playback)
- `sidplayer2`: CIA2 Timer A NMI playback now bootstraps and tracks a varying period, fixing NMI-driven tunes whose music runs entirely from the NMI handler (e.g. Rob Hubbard's `BMX Kidz`, subtune 1). `detectNMI()` now activates when the NMI is enabled with a valid handler even if the tune left CIA2 Timer A unprogrammed (it relies on the C64 power-on latch of `$FFFF` to fire the first NMI); the audio callback re-reads the handler-reprogrammed latch after each NMI so sequenced/variable NMI rates track correctly; and `executeNMI()` now forwards all `$D400–$D41F` writes to the engine (previously only the `$D418` digi byte was captured), so voice, filter and control writes made from the NMI are heard
- Verified no regressions across `Skate or Die` (NMI digi), `Arkanoid` (secondary IRQ), `Commando`, `Delta`, `Monty on the Run`, `Sanxion`, `Comic Bakery`, and `Times of Lore`

## [1.9.3] - 2026-07-08

### Fixed
- `sidplayer2`: Play routine is again invoked as a subroutine instead of via a synthetic IRQ return frame, fixing tunes that crashed into silence after a few frames (e.g. `Rambo: First Blood Part II` by Martin Galway). The 3-byte IRQ frame injection added in 1.9.0 left an extra byte on the stack; Galway-style players that use RTS-based dispatch (with a deliberate JSR/RTS imbalance) then read a wrong return address, land mid-instruction, and fall into an infinite `BRK` loop. The frame trigger now sets `SP=0xFF`, sets the I flag, and jumps to the play address; completion is detected when the routine pops past the empty stack (`CPU()` returns `>= 0xFE`), matching libsidplayfp's driver (`psiddrv.a65` `irqjob`: `jsr play`, where the play routine ends in RTS/RTI and the RTI lives in the driver wrapper, not the play routine)
- `sidplayer2`: Secondary IRQ chaining (Arkanoid raster trick) enters the secondary handler as a subroutine too, for the same reason
- Verified no regressions: `Skate or Die` (NMI digi), `Arkanoid` (secondary IRQ), `Commando`, and `Delta` all still play

## [1.9.1] - 2026-03-10

### Fixed
- `sidplayer2`: Restored `clk_ratio + 1.0` cycle count for `clock()` calls, ensuring exactly one audio sample is generated per callback slot; the fractional-accumulation replacement caused `floor(22/22.34) = 0` samples ~66% of the time, producing severe choppy and muffled output
- `sidplayer2`: Volume scaling moved to `sidInstance->setVolume(0.25f)` (called once after `reset()`); redundant inline `sample * 0.25f` cast, manual int clamp, and `int16_t` conversion removed from the audio callback hot path since `clock()` handles clamping and conversion internally

## [1.9.0] - 2026-03-10

### Changed
- `sidplayer2`: PAL/NTSC C64 model now detected from SID file header byte `0x77` bits 2–3 (`0x08` = NTSC, otherwise PAL); CPU clock (`985248 Hz` PAL / `1022727 Hz` NTSC) and frame rate (`50 Hz` / `60 Hz`) are set accordingly, fixing NTSC tunes that previously played sharp by ~3.8%
- Added `C64_NTSC_CPUCLK = 1022727.0` and `NTSC_FRAMERATE = 60.0` constants to `reSIDuEngine.h`
- `sidplayer2`: Frame trigger now performs full hardware IRQ simulation — a 3-byte return frame (PCL/PCH of `0xEA31`, status with I-flag set) is pushed onto the stack before jumping to the play address, and `ST` has the interrupt-disable flag set, matching the CPU state a real C64 interrupt handler receives; previously the play routine was called as a bare subroutine with an empty stack
- `sidplayer2`: IRQ return detection (`0xEA31`/`0xEA81`) no longer requires ROM to be banked in; the ROM-bank guard was removed since the return address is now always our injected sentinel
- `sidplayer2`: Secondary IRQ chaining (e.g. Arkanoid raster trick) uses the same IRQ injection as the primary frame trigger
- `sidplayer2`: Removed leftover debug `printf` calls (`DBG init`, D418 write log, secondary IRQ fire log) and their associated static counters

## [1.8.0] - 2026-03-05

### Changed
- Renamed cryptic and reused variable names throughout `reSIDuEngine.cpp` and `reSIDuEngine.h` for clarity: `tmp` in `processSID()` split into purpose-specific names (`triggerImmediateADSR`, `lfsrState`, `bandLimitStep`, `phase16`, `ringModPhase`, `hpOut`, `bpOut`, `lpOut`); math-notation names in `buildKinkedDacTable()` replaced (`_2R`→`twoR`, `Vn`→`nodeVoltage`, `Rn`→`nodeResistance`, `I`→`nodeCurrent`, `Vsum`→`voltageSum`); `combiwf`→`avgWF`, `wfArray`→`waveformTable`, `bitlevel`→`bitInfluence`, `SR`→`sustainReleaseReg`, `ctrl`→`controlReg`, `w0lp`/`w0hp`→`extLPCoeff`/`extHPCoeff`

## [1.7.0] - 2026-03-05

### Changed
- Waveform floating-output decay (bitfade) implemented: when voice waveform bits are set to 0, `previousWFOut` now gradually decays (`output &= output >> 1`) rather than holding the last value indefinitely, matching the analog capacitor discharge behaviour of real hardware. Initial hold periods and fade intervals are chip-specific (MOS6581: 54 000 / 1 400 CPU cycles; MOS8580: 800 000 / 50 000 CPU cycles)
- MOS8580 OSC3 readback delayed by one sample for triangle/sawtooth waveforms via a `triSawPipeline` register, matching the hardware's half-cycle output latency
- Data bus capacitance modelled: `busValue` and `busValueTtl` track the last byte driven onto the SID data bus. Register writes refresh the TTL (MOS6581: 7 424 cycles; MOS8580: 663 552 cycles); reads of write-only registers return the decaying bus remnant and halve the remaining TTL; reads of OSC3/ENV3 refresh the TTL. Bus discharges to 0x00 when TTL expires
- ENV3 register ($D41C) now reflects the envelope counter value from the start of the current sample (phi1 latch), matching the hardware timing where ENV3 is sampled before ADSR updates execute
- `sidplayer2`: OSC3 ($D41B) and ENV3 ($D41C) register values are now synced from the emulator into `memory[]` after each audio sample, so C64 music code that reads these addresses (e.g. for random number generation or voice synchronisation) receives correct emulated values instead of zero

## [1.6.0] - 2026-03-04

### Changed
- Oscillator waveform and envelope outputs now pass through chip-specific R-2R ladder DAC lookup tables (`oscDAC6581/8580[4096]`, `envDAC6581/8580[256]`) instead of ideal linear conversion, accurately reproducing DAC nonlinearity in both the waveform and amplitude paths
- MOS6581 oscillator DAC additionally models the NMOS source-follower output stage with a cubic saturation curve (residfpII model: `1.1V − 0.121V³`), adding the warm harmonic distortion characteristic of the original chip at higher amplitudes
- Filter cutoff mapping replaced with hardware-accurate R-2R ladder DAC simulation (`buildKinkedDacTable`), producing a precomputed 2048-entry lookup table per chip model instead of a single exponential scalar
- MOS6581 DAC modelled with 2R/R=2.20 and missing bit-0 termination resistor, accurately reproducing the chip's non-linear, compressed low-end frequency response
- MOS8580 DAC modelled with 2R/R=2.00 and correct termination, producing a near-linear response that matches the old formula to within 0.5% across the full FC range
- 6581 cutoff table calibrated to 16 kHz maximum (hardware-measured upper bound), producing the same mid-range coefficient as the previous formula at FC_HI=100 (within 1.4%)
- 6581 cutoff table monotonized after construction: the kinked DAC produces genuine non-monotonic steps at every FC_LO roll-over (fc=8,16,24,…) due to the compressed lower-bit weights; real hardware smooths these analogically, so the table is post-processed to prevent audible bumps in filter sweeps
- MOS6581 resonance corrected to hardware-measured formula `(~res & 0xf) / 8.0` (upper nibble of RES_FILT register only), replacing the previous incorrect comparison against the full register byte; clamped at 0.3 (Q≈3.3) to prevent SVF numerical instability at near-self-oscillation settings
- MOS8580 resonance unchanged: `pow(2, (4 - res) / 8)`, Q range 0.71–2.59
- External RC output stage added after the SVF: lowpass at ~16 kHz (10 kΩ / 1000 pF) removes near-Nyquist content; highpass at ~1.6 Hz (10 kΩ / 10 µF) removes DC drift from the filter integrators
- Soft-clip applied to final output: `tanh`-based compression engages above ±1.0 with 0.5 headroom, preventing harsh digital peaks at high voice counts or resonance settings

## [1.5.0] - 2026-03-04

### Changed
- ADSR rate counter replaced with hardware-accurate 15-bit LFSR simulation: periods are now derived by counting LFSR clocks from 0x7fff to the rate-specific comparison value, matching the real SID timing circuit
- ADSR rate counter now uses a `while` loop to fire multiple envelope steps per audio sample for fast rates (e.g. rate 0 fires ~3×/sample), eliminating missed steps at short periods
- Exponential decay prescaler table corrected to hardware-measured thresholds (`0x5d`, `0x36`, `0x1a`, `0x0e`, `0x06`) and correct period assignments (high envelope = fast decay, low envelope = slow decay)
- Removed `adsrStep` array; envelope always increments/decrements by 1 per LFSR firing, with fast rates handled by multiple firings per sample

## [1.4.0] - 2026-03-04

### Changed
- Phase accumulator converted from `double` to `uint32_t` with 24-bit masking (`& 0xFFFFFF`), matching hardware behavior
- Fractional phase carry (`phaseAccumulatorFrac`) preserves pitch accuracy when advancing the accumulator once per output sample
- MSB rising-edge detection for hard sync now uses the XOR method `(~old & new) & 0x800000` for correct 0→1 transition detection

## [1.3.0] - 2026-02-12

### Fixed
- SID player: Read-modify-write CPU instructions (INC, DEC, ASL, LSR, ROL, ROR) on SID registers now correctly forwarded to the engine, fixing silent playback of tunes that use RMW instructions for register updates (e.g., Fred Gray tunes)
- SID player: Writes to SID mirror addresses ($D420-$D7FF) now forwarded to the engine, not just mirrored in memory
- SID player: Zero-page indexed addressing modes now correctly wrap within the zero page, matching real 6502 behavior

## [1.2.0] - 2026-02-11

### Fixed
- Gate-off/gate-on transitions within the same sample period are no longer lost, fixing missed notes in SID tunes that retrigger voices rapidly

## [1.1.0] - 2026-01-08

### Added
- Dedicated `pulseTriangle8580` lookup table for accurate pulse+triangle combined waveform emulation
- `getMeasuredCombinedWF()` function implementing frequency-dependent filtering for combined waveforms
- Enhanced combined waveform algorithm based on real SID chip measurements

### Fixed  
- Pulse+triangle combination now uses correct dedicated lookup table instead of `pulseSaw8580`
- Corrected waveform generation for pulse+triangle to match real SID analog behavior
- Enhanced accuracy of combined waveforms through proper bit-interaction modeling

### Changed
- Combined waveform generation now uses improved frequency-dependent filtering for better authenticity
- Frequency-dependent filtering applied to combined waveforms for more accurate frequency response
- Pulse+triangle parameters tuned to match measured SID chip behavior

## [1.0.0] - Initial Release

### Features
- Complete SID chip emulation (MOS 6581/8580)
- 3 independent voices with ADSR envelopes
- 4 waveform types: triangle, sawtooth, pulse, noise
- Combined waveform support for complex timbres
- Multi-mode filter (lowpass, bandpass, highpass)
- Ring modulation and hard sync between voices
- Band-limited synthesis to reduce aliasing
- Configurable sample rate and clock frequency
- reSIDfp API compatibility
- Multiple audio backend examples (SDL2, PortAudio, miniaudio)