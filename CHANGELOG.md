# Changelog

All notable changes to reSIDuEngine will be documented in this file.

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