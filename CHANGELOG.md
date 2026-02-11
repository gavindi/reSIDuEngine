# Changelog

All notable changes to reSIDuEngine will be documented in this file.

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