// reSIDuEngine - C++ implementation of the Commodore 64 SID chip

/*
 * Copyright (c) 2026, Gavin Graham <gavindi@gmail.com>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public License, v. 2.0.
 * If a copy of the MPL was not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include "reSIDuEngine.h"
#include <cstring>
#include <algorithm>

namespace reSIDuEngine
{

/**
 * External definitions for static constexpr array members.
 * In C++14 and earlier, static constexpr class members that are ODR-used
 * (one-definition-rule-used, meaning their address is taken or they're passed
 * by reference) must have an external definition in addition to their in-class
 * declaration. Without these definitions, the linker will produce "undefined reference"
 * errors. C++17 relaxed this requirement with inline variables, but we're targeting C++14.
 */
constexpr std::array<int, 3> SID::FILTSW;

/**
 * SID Constructor
 *
 * Initializes a SID emulator instance with the specified sample rate and chip model.
 * This constructor performs complete initialization of all SID subsystems:
 *
 * 1. Clock and timing initialization:
 *    - Sets up the sample rate for audio output
 *    - Configures PAL CPU clock and frame rate
 *    - Calculates the clock ratio (CPU cycles per sample)
 *
 * 2. Filter coefficient calculation:
 *    - Computes cutoff frequency ratios for both 6581 and 8580 models
 *    - These ratios are sample-rate-dependent and used in the bi-quad filter
 *
 * 3. ADSR envelope timing tables:
 *    - Builds period tables for all 16 attack/decay/release rates
 *    - Scales periods based on sample rate for correct timing
 *    - Initializes step sizes (how much envelope changes per period)
 *    - Creates exponential decay table for logarithmic envelope behavior
 *
 * 4. Combined waveform tables:
 *    - Generates lookup tables for analog waveform interactions
 *    - Tri+Saw, Pulse+Saw, and Pulse+Tri+Saw combinations
 *    - Models the complex bit-fighting behavior of the SID's DAC
 *
 * 5. SID state initialization:
 *    - Clears all registers
 *    - Resets all voices to silent state
 *    - Initializes phase accumulators, envelopes, and filters
 */
SID::SID(double sampleRate, SIDModel model)
    : sampleRate(sampleRate)
    , cpuClock(C64_PAL_CPUCLK)
    , frameRate(PAL_FRAMERATE)
    , sidModel(model)
    , volume(1.0f)
    , filterEnabled(true)
    , externalInput(0)
    , previousLowpass(0)
    , previousBandpass(0)
{
    // Initialize voice mute flags (all voices unmuted by default)
    voiceMuted.fill(false);
    // Zero-initialize all 32 SID registers
    SIDRegister.fill(0);

    // Calculate the clock ratio: how many CPU cycles occur per audio sample
    // This is critical for pitch accuracy - it determines phase accumulator advancement
    updateClockRatio();

    // Precalculate filter cutoff ratios for both chip models
    // These are used in the bi-quad filter equations and depend on sample rate
    // 8580: cutoff range approximately 0-12.5 kHz
    // 6581: cutoff range approximately 0-20 kHz (but with non-linear behavior)
    cutoffRatio8580 = -2.0 * M_PI * (12500.0 / 256.0) / sampleRate;
    cutoffRatio6581 = -2.0 * M_PI * (20000.0 / 256.0) / sampleRate;

    // Build the ADSR period table
    // Period 0 is special: it must be at least 9 samples to avoid divide-by-zero
    // in the band-limiting calculations (which divide by accumulatorAdd which depends on clkRatio)
    float period0 = std::max(clkRatio, 9.0);

    // The remaining 15 periods are based on the original SID timing
    // These values are in CPU cycles and are scaled by clkRatio to convert to samples
    // The periods roughly double for each successive rate value, giving exponential timing
    adsrPeriods = {
        period0,    // Rate 0: fastest (special case)
        32.0f,      // Rate 1: 2 ms
        63.0f,      // Rate 2: 8 ms
        95.0f,      // Rate 3: 16 ms
        149.0f,     // Rate 4: 24 ms
        220.0f,     // Rate 5: 38 ms
        267.0f,     // Rate 6: 56 ms
        313.0f,     // Rate 7: 68 ms
        392.0f,     // Rate 8: 80 ms
        977.0f,     // Rate 9: 240 ms
        1954.0f,    // Rate 10: 750 ms
        3126.0f,    // Rate 11: 1.5 s
        3907.0f,    // Rate 12: 2.4 s
        11720.0f,   // Rate 13: 3 s
        19532.0f,   // Rate 14: 9 s
        31251.0f    // Rate 15: 24 s
    };

    // Initialize step sizes for ADSR rates
    // Rate 0 gets a special step size to compensate for its short period
    // All other rates use step=1 (increment/decrement envelope by 1 each period)
    adsrStep[0] = static_cast<int>(std::ceil(period0 / 9.0));
    for (int i = 1; i < 16; i++)
        adsrStep[i] = 1;

    // Build the ADSR exponential table
    // This table implements the logarithmic decay characteristic of the SID's release
    // The table contains prescaler values: the envelope counter only advances every N
    // period expirations, where N depends on the current envelope level
    // As the envelope decays, the prescaler increases, slowing the decay rate
    // This creates the characteristic exponential decay curve
    for (int i = 0; i < 256; i++)
    {
        if (i < 6)
            adsrExptable[i] = 1;      // Very high levels: no slowdown
        else if (i < 14)
            adsrExptable[i] = 30;     // High levels: 30x slower
        else if (i < 26)
            adsrExptable[i] = 16;     // Medium-high: 16x slower
        else if (i < 54)
            adsrExptable[i] = 8;      // Medium: 8x slower
        else if (i < 93)
            adsrExptable[i] = 4;      // Medium-low: 4x slower
        else if (i < 93 + 62)
            adsrExptable[i] = 2;      // Low: 2x slower
        else
            adsrExptable[i] = 1;      // Very low: no slowdown
    }

    // Create combined waveform lookup tables
    // These model the complex analog behavior when multiple waveform bits are enabled
    // The SID's DAC uses open-drain drivers and FET switches that cause bit-fighting
    // when multiple waveforms try to drive the output simultaneously
    // Each combination has different parameters to match measured behavior

    // Triangle + Sawtooth: moderate interaction, medium threshold
    createCombinedWF(triSaw8580, 0.8, 2.4, 0.64);

    // Pulse + Sawtooth: strong interaction, high threshold
    createCombinedWF(pulseSaw8580, 1.4, 1.9, 0.68);

    // Pulse + Triangle: moderate interaction, medium threshold (based on libsidplayfp measurements)
    createCombinedWF(pulseTriangle8580, 1.2, 2.2, 0.66);

    // Pulse + Triangle + Sawtooth: moderate interaction, medium threshold
    createCombinedWF(pulseTriSaw8580, 0.8, 2.5, 0.64);

    // Perform initial SID state initialization
    // Clears all registers and resets all voice/filter state
    initSID();
}

/**
 * SID Destructor
 *
 * Currently empty as we use automatic storage (std::array) for all data.
 * No manual memory management needed. This is here for future expansion
 * and to follow good C++ practice of declaring virtual destructors.
 */
SID::~SID()
{
}

/**
 * Initialize SID state to power-on defaults.
 *
 * This function resets the SID to a clean state as if power was just applied.
 * It's called by the constructor and can be called manually via reset() to
 * restart emulation with a clean slate.
 *
 * What gets reset:
 * 1. All SID registers (0xD400-0xD7FF and mirror at 0xDE00-0xDFFF)
 * 2. ADSR envelope state (state machine, counters, exponential counters)
 * 3. Phase accumulators (oscillator positions reset to zero)
 * 4. Noise LFSR (reset to initial seed value)
 * 5. Waveform history (for floating DAC and combined waveform interpolation)
 * 6. Filter state (lowpass and bandpass integrator states)
 * 7. MSB tracking (for sync and ring modulation detection)
 */
void SID::initSID()
{
    // Clear all 32 SID registers (0xD400-0xD41F)
    SIDRegister.fill(0);

    // Initialize per-voice state for all 3 voices
    for (int i = 0; i < SID_CHANNELS; i++)
    {
        // Set ADSR to the HOLDZERO state (envelope at zero, waiting for gate)
        adsrState[i] = HOLDZERO_BITMASK;

        // Reset ADSR timing counters
        rateCounter[i] = 0;           // Rate period counter
        envelopeCounter[i] = 0;       // Current envelope level (0-255)
        exponentCounter[i] = 0;       // Exponential decay prescaler counter
        previousSR[i] = 0;            // Previous sustain/release register value

        // Reset oscillator state
        phaseAccumulator[i] = 0;      // Current phase (0 to 0xFFFFFF)
        previousAccumulator[i] = 0;   // Previous phase (for detecting transitions)

        // Reset noise generator to initial LFSR seed
        // 0x7FFFF8 is a non-zero seed that produces good pseudo-random sequences
        noiseLFSR[i] = 0x7FFFF8;

        // Reset waveform history
        previousWFOut[i] = 0;         // For emulating floating DAC when waveform is 00
        previousWaveData[i] = 0;      // For combined waveform interpolation
    }

    // Initialize filter state
    previousLowpass = 0;     // Lowpass integrator output
    previousBandpass = 0;    // Bandpass integrator output

    // Reset external audio input (EXT IN)
    externalInput = 0;

    // Initialize sync detection state
    sourceMSBrise.fill(0);   // MSB rising edge flags for hard sync
    sourceMSB.fill(0);       // Previous MSB values for ring modulation
}

/**
 * Public reset function.
 *
 * Allows external code to reset the SID to power-on state.
 * Simply delegates to initSID() which does all the work.
 * Use this when loading a new song or when you need to ensure clean state.
 */
void SID::reset()
{
    initSID();
}

/**
 * Update the clock ratio and derived timing values.
 *
 * The clock ratio is fundamental to the emulation - it determines how much
 * to advance the phase accumulators per audio sample, which directly affects pitch.
 *
 * clkRatio = CPU cycles per audio sample = cpuClock / sampleRate
 *
 * For example, with PAL clock (985248 Hz) and 44100 Hz sample rate:
 * clkRatio = 985248 / 44100 ≈ 22.34 CPU cycles per sample
 *
 * The frame sample period is used for frame-synchronized timing:
 * frameSamplePeriod = samples per frame = sampleRate / frameRate
 *
 * For example, at 44100 Hz with PAL 50 Hz frame rate:
 * frameSamplePeriod = 44100 / 50 = 882 samples per frame
 */
void SID::updateClockRatio()
{
    clkRatio = cpuClock / sampleRate;
    frameSamplePeriod = sampleRate / frameRate;
}

/**
 * Set the frame rate for frame-synchronized timing.
 *
 * Updates the frame rate and recalculates the frame sample period.
 * This is used when SID music players rely on frame-based timing
 * (calling the player routine at a fixed rate like 50 or 60 Hz).
 *
 * @param hz Frame rate in Hz (typically 50 for PAL, ~60 for NTSC)
 */
void SID::setFramerate(double hz)
{
    frameRate = hz;
    frameSamplePeriod = sampleRate / frameRate;
}

/**
 * Set the SID chip model.
 *
 * Changes between MOS6581 (original) and MOS8580 (revised) emulation.
 * This affects filter behavior, combined waveform output, and overall tone.
 * Can be called at any time, though changing during playback may cause artifacts.
 *
 * @param model The chip model to emulate (MOS6581 or MOS8580)
 */
void SID::setModel(SIDModel model)
{
    sidModel = model;
}

/**
 * Write a byte to a SID register.
 *
 * This is the primary interface for controlling the SID. All sound generation
 * parameters are set by writing to memory-mapped registers:
 *
 * Voice 1: 0xD400-0xD406
 * Voice 2: 0xD407-0xD40D
 * Voice 3: 0xD40E-0xD414
 * Filter:  0xD415-0xD418
 * Misc:    0xD419-0xD41C
 *
 * @param addr 16-bit address (only bits 0-4 used for 32 registers)
 * @param value 8-bit value to write to the register
 */
void SID::write(int addr, unsigned char value)
{
    // Map address to register index (0-31)
    // Accept addresses in multiple forms for compatibility:
    // - Full address (0xD400-0xD41F) - legacy reSIDuEngine
    // - Register offset (0-31) - reSIDfp API
    SIDRegister[addr & 0x1F] = value;
}

/**
 * Read a byte from a SID register.
 *
 * Most SID registers are write-only, but a few can be read:
 * - 0xD41B (offset 27): Voice 3 oscillator output (upper 8 bits)
 * - 0xD41C (offset 28): Voice 3 envelope output (ADSR level)
 *
 * These are often used for random number generation and music synchronization.
 *
 * @param addr Address or offset to read from (only bits 0-4 used for 32 registers)
 * @return 8-bit value at that address (0 for write-only registers)
 */
unsigned char SID::read(int addr)
{
    // Map address to register index (0-31)
    return SIDRegister[addr & 0x1F];
}

/**
 * Create a combined waveform lookup table.
 *
 * This function models the complex analog behavior of the SID when multiple
 * waveform generator bits are enabled simultaneously. Unlike digital mixing,
 * the SID's waveforms interact through analog circuitry:
 *
 * 1. Each waveform generator drives a set of bits on the DAC
 * 2. The drivers are open-drain (or open-collector) with pull-up resistors
 * 3. When multiple waveforms try to drive the same bit to different values,
 *    they "fight" through the resistor network
 * 4. This creates complex bit interactions that depend on neighboring bits
 * 5. The result is not a simple OR, AND, or XOR - it's analog bit-fighting
 *
 * The algorithm simulates this by:
 * - For each output bit position j, calculate influence from all input bits k
 * - Influence decreases exponentially with distance: 1/bitstrength^|k-j|
 * - Influence is scaled by bitmul and by whether input bit k is set
 * - If total influence exceeds threshold, output bit j is set
 *
 * Different waveform combinations have different parameters to match measurements
 * from real SID chips.
 *
 * @param waveformArray Reference to 4096-entry array to fill (12-bit waveform space)
 * @param bitmul Overall scaling factor for bit influence strength
 * @param bitstrength Exponential decay factor for influence vs distance
 * @param threshold Level above which an output bit is considered "on"
 */
void SID::createCombinedWF(std::array<uint16_t, 4096>& waveformArray,
                             double bitmul, double bitstrength, double threshold)
{
    // Iterate through all possible 12-bit input combinations
    for (int i = 0; i < 4096; i++)
    {
        waveformArray[i] = 0;

        // For each output bit position
        for (int j = 0; j < 12; j++)
        {
            double bitlevel = 0;

            // Calculate influence from all input bits
            for (int k = 0; k < 12; k++)
            {
                // Influence decreases with distance between bits j and k
                // Also depends on whether input bit k is set (centered at 0.5)
                bitlevel += (bitmul / std::pow(bitstrength, std::abs(k - j))) *
                           (((i >> k) & 1) - 0.5);
            }

            // If accumulated influence exceeds threshold, set output bit j
            if (bitlevel >= threshold)
                waveformArray[i] += static_cast<uint16_t>(std::pow(2, j));
        }

        // Scale up by 12 to match the range of normal waveforms
        // This compensates for the bit-fighting reducing overall amplitude
        waveformArray[i] *= 12;
    }
}

/**
 * Look up a combined waveform value with interpolation.
 *
 * Retrieves a value from a combined waveform lookup table and applies
 * simple averaging with the previous sample to smooth transitions.
 * Also handles the different behavior of 6581 vs 8580 chips.
 *
 * On the 6581, many combined waveforms produce different output than on the 8580.
 * Specifically, certain combinations effectively have only 11 significant bits
 * on the 6581 (the upper bit is often zero), making them quieter.
 *
 * @param voiceIndex Voice number requesting the waveform (0-2)
 * @param waveformArray Combined waveform lookup table to use
 * @param index 12-bit index into the table (from phase accumulator bits)
 * @param differ6581 If true, apply 6581-specific behavior (mask off MSB)
 * @return 16-bit waveform output value
 */
uint16_t SID::combinedWF(int voiceIndex, const std::array<uint16_t, 4096>& waveformArray,
                           int index, bool differ6581)
{
    // On 6581, many combined waveforms are effectively halved (11-bit instead of 12-bit)
    // This masks off the MSB, reducing the output by half
    if (differ6581 && sidModel == MOS6581)
        index &= 0x7FF;  // Keep only lower 11 bits

    // Simple averaging with previous sample to smooth waveform transitions
    // This helps reduce discontinuities when the waveform changes rapidly
    uint16_t combiwf = (waveformArray[index] + previousWaveData[voiceIndex]) / 2;

    // Store current value for next sample's interpolation
    previousWaveData[voiceIndex] = waveformArray[index];

    return combiwf;
}

/**
 * Get measured combined waveform value using libsidplayfp-style approach.
 *
 * This function implements the libsidplayfp approach for combined waveforms,
 * which uses direct table lookups with frequency-dependent filtering and
 * previous value feedback to model the complex analog behavior of the SID chip.
 *
 * The algorithm models:
 * 1. Frequency-dependent filtering (analog RC circuit behavior)
 * 2. Previous value feedback (capacitive coupling)
 * 3. Chip-specific differences (6581 vs 8580)
 *
 * @param voiceIndex Voice number (0-2)
 * @param index 12-bit waveform index from phase accumulator
 * @param waveformCtrl Waveform control register bits (upper nibble)
 * @return Combined waveform output value as 16-bit unsigned integer
 */
uint16_t SID::getMeasuredCombinedWF(int voiceIndex, int index, uint8_t waveformCtrl)
{
    // Select the appropriate lookup table based on waveform combination
    const std::array<uint16_t, 4096>* wfArray = nullptr;
    
    switch (waveformCtrl & 0xF0) {
        case (TRI_BITMASK | SAW_BITMASK):
            wfArray = &triSaw8580;
            break;
        case (PULSE_BITMASK | TRI_BITMASK):
            wfArray = &pulseTriangle8580;
            break;
        case (PULSE_BITMASK | SAW_BITMASK):
            wfArray = &pulseSaw8580;
            break;
        case (PULSE_BITMASK | TRI_BITMASK | SAW_BITMASK):
            wfArray = &pulseTriSaw8580;
            break;
        default:
            // Should never happen - return 0 for safety
            return 0;
    }
    
    // Get pitch for frequency-dependent filtering
    // Extract frequency from voice registers (assuming 16-bit frequency value)
    const uint8_t* voiceRegister = &SIDRegister[voiceIndex * 7];
    uint16_t freq = voiceRegister[0] | (voiceRegister[1] << 8);
    uint8_t pitch = (freq >> 8) ? (freq >> 8) : 1; // Avoid division by zero
    
    // Apply frequency-dependent filtering (similar to libsidplayfp's 0x7777 + 0x8888/pitch)
    // This models how the analog circuitry's response varies with frequency
    uint16_t filterCoeff = 0x7777 + (0x8888 / pitch);
    
    // Get table value
    uint16_t tableValue = (*wfArray)[index >> 4]; // Scale 12-bit to 8-bit table index
    
    // Apply frequency-dependent filtering with previous value feedback
    // This models the capacitive coupling and frequency response of the analog circuit
    uint32_t filteredValue = (tableValue * filterCoeff + 
                             previousWaveData[voiceIndex] * (0xFFFF - filterCoeff)) >> 16;
    
    // Store current value for next sample's feedback
    previousWaveData[voiceIndex] = tableValue;
    
    // Apply 6581-specific behavior (reduced bit depth for some combinations)
    if (sidModel == MOS6581 && (waveformCtrl & PULSE_BITMASK)) {
        // On 6581, pulse combinations often have reduced bit depth
        filteredValue &= 0x7FFF; // Mask off MSB (11-bit instead of 12-bit)
    }
    
    return static_cast<uint16_t>(filteredValue) << 8; // Scale back to 16-bit range
}

/**
 * Process the SID chip and return its mixed output.
 *
 * This is the core emulation function. It processes all 3 voices of the SID chip
 * for one sample period. The function is large and complex because it handles:
 *
 * 1. ADSR envelope generation (Attack/Decay/Sustain/Release)
 * 2. Waveform generation (Triangle, Sawtooth, Pulse, Noise)
 * 3. Special effects (Sync, Ring Modulation)
 * 4. Combined waveforms (multiple waveform bits set)
 * 5. Band-limited synthesis (anti-aliasing)
 * 6. Filter routing
 * 7. Multi-mode filter processing (Lowpass/Bandpass/Highpass)
 * 8. Voice mixing
 *
 * The function processes voices in order (0, 1, 2), which is important because
 * voices can modulate each other through sync and ring modulation.
 *
 * @return Mixed audio output for the SID chip
 */
float SID::processSID()
{
    float filterInput = 0;   // Accumulator for signals routed to filter
    float output = 0;        // Accumulator for final output (filtered + unfiltered)

    // Process all 3 voices
    for (int voiceIndex = 0; voiceIndex < SID_CHANNELS; voiceIndex++)
    {
        // Get base pointer to this voice's registers (7 consecutive registers per voice)
        // Voice 0: SIDRegister[0-6], Voice 1: SIDRegister[7-13], Voice 2: SIDRegister[14-20]
        const uint8_t* voiceRegister = &SIDRegister[voiceIndex * 7];

        // Read control register: contains waveform, gate, sync, ring, test bits
        uint8_t ctrl = voiceRegister[4];
        uint8_t waveformCTRL = ctrl & 0xF0;  // Waveform bits (upper nibble)
        uint8_t test = ctrl & TEST_BITMASK;  // Test bit (halts oscillator)

        // Read sustain/release register
        uint8_t SR = voiceRegister[6];

        int tmp = 0;  // Multipurpose temporary variable

        // Check previous gate state to detect gate transitions
        uint8_t previousGate = adsrState[voiceIndex] & GATE_BITMASK;

        // =====================================================================
        // ADSR ENVELOPE GENERATOR
        // =====================================================================
        // The SID's ADSR is a 4-stage envelope generator implemented as a
        // state machine with an 8-bit counter (0-255) representing amplitude.
        // States: Attack (ramping up), Decay (ramping down to sustain),
        //         Sustain (holding at level), Release (ramping to zero)

        // Detect gate bit transitions (note on/off events)
        if (previousGate != (ctrl & GATE_BITMASK))
        {
            if (previousGate)
            {
                // Gate falling edge (note-off): enter Release phase
                // Clear gate, attack, and decay/sustain state bits
                adsrState[voiceIndex] &= 0xFF - (GATE_BITMASK | ATTACK_BITMASK | DECAYSUSTAIN_BITMASK);
            }
            else
            {
                // Gate rising edge (note-on): enter Attack phase
                // Set gate, attack, and decay/sustain bits (decay/sustain comes after attack)
                adsrState[voiceIndex] = GATE_BITMASK | ATTACK_BITMASK | DECAYSUSTAIN_BITMASK;

                // Workaround for SR->GATE write order issue
                // If sustain/release was just written with a higher value, trigger
                // an immediate envelope update for crisp attack starts
                if ((SR & 0xF) > (previousSR[voiceIndex] & 0xF))
                    tmp = 1;
            }
        }

        // Remember current SR value for next sample's comparison
        previousSR[voiceIndex] = SR;

        // Advance the rate counter
        // This counter determines when to update the envelope level
        // It increments by clkRatio each sample and is compared against the period
        rateCounter[voiceIndex] += clkRatio;

        // Handle rate counter wraparound (ADSR delay bug emulation)
        // The real SID's 15-bit rate counter can wrap around, causing timing glitches
        if (rateCounter[voiceIndex] >= 0x8000)
            rateCounter[voiceIndex] -= 0x8000;

        // Determine which ADSR phase we're in and select the appropriate rate
        float period;  // How many samples between envelope updates
        int step;      // How much to change envelope per update

        if (adsrState[voiceIndex] & ATTACK_BITMASK)
        {
            // Attack phase: use attack rate (upper nibble of AD register)
            step = voiceRegister[5] >> 4;
            period = adsrPeriods[step];
        }
        else if (adsrState[voiceIndex] & DECAYSUSTAIN_BITMASK)
        {
            // Decay phase: use decay rate (lower nibble of AD register)
            step = voiceRegister[5] & 0xF;
            period = adsrPeriods[step];
        }
        else
        {
            // Release phase: use release rate (lower nibble of SR register)
            step = SR & 0xF;
            period = adsrPeriods[step];
        }

        // Convert rate index to step size
        step = adsrStep[step];

        // Check if the rate counter has reached the period (time to update envelope)
        // Also check we're not in the middle of a gate transition (tmp != 0)
        if (rateCounter[voiceIndex] >= period &&
            rateCounter[voiceIndex] < period + clkRatio &&
            tmp == 0)
        {
            // Reset rate counter (subtract period to preserve fractional accumulation)
            rateCounter[voiceIndex] -= period;

            // Update the exponential counter
            // During Attack, always update (linear attack)
            // During Decay/Release, only update when exponential prescaler expires
            if ((adsrState[voiceIndex] & ATTACK_BITMASK) ||
                ++exponentCounter[voiceIndex] == adsrExptable[envelopeCounter[voiceIndex]])
            {
                // Only update envelope if not holding at zero
                if (!(adsrState[voiceIndex] & HOLDZERO_BITMASK))
                {
                    if (adsrState[voiceIndex] & ATTACK_BITMASK)
                    {
                        // Attack: increment envelope toward maximum (255)
                        envelopeCounter[voiceIndex] += step;
                        if (envelopeCounter[voiceIndex] >= 0xFF)
                        {
                            // Reached maximum: enter Decay phase
                            envelopeCounter[voiceIndex] = 0xFF;
                            adsrState[voiceIndex] &= 0xFF - ATTACK_BITMASK;
                        }
                    }
                    else if (!(adsrState[voiceIndex] & DECAYSUSTAIN_BITMASK) ||
                             envelopeCounter[voiceIndex] > (SR >> 4) + (SR & 0xF0))
                    {
                        // Decay or Release: decrement envelope
                        // During Decay, only decrement if above sustain level
                        // During Release, always decrement
                        envelopeCounter[voiceIndex] -= step;

                        // Check if we've reached zero
                        if (envelopeCounter[voiceIndex] <= 0 && envelopeCounter[voiceIndex] + step != 0)
                        {
                            // Envelope reached zero: enter hold-at-zero state
                            envelopeCounter[voiceIndex] = 0;
                            adsrState[voiceIndex] |= HOLDZERO_BITMASK;
                        }
                    }
                }

                // Reset exponential counter
                exponentCounter[voiceIndex] = 0;
            }
        }

        // Ensure envelope counter stays within 8-bit range (0-255)
        // The signed arithmetic above can cause temporary over/underflow
        envelopeCounter[voiceIndex] &= 0xFF;

        // =====================================================================
        // OSCILLATOR (PHASE ACCUMULATOR)
        // =====================================================================
        // The oscillator is a 24-bit phase accumulator that increments by the
        // frequency value each CPU cycle. The upper bits of this accumulator
        // are used to generate various waveforms.

        // Calculate phase accumulator increment for this sample
        // Frequency is 16-bit (registers 0-1), scaled by clock ratio
        // accumulatorAdd is in units of "phase increments per sample"
        double accumulatorAdd = (voiceRegister[0] | (voiceRegister[1] << 8)) * clkRatio;

        // Update phase accumulator with sync and test handling
        // For sync: voice 0 syncs to voice 2, voice 1 to voice 0, voice 2 to voice 1
        int syncSource = (voiceIndex + 2) % SID_CHANNELS;
        if (test || ((ctrl & SYNC_BITMASK) && sourceMSBrise[syncSource]))
        {
            // Test bit or hard sync: reset phase to zero
            // Hard sync resets this oscillator when the sync source oscillator's
            // phase wraps around (MSB goes from 1 to 0, detected as rising edge)
            phaseAccumulator[voiceIndex] = 0;
        }
        else
        {
            // Normal operation: advance phase accumulator
            // We use double precision to preserve fractional phase and avoid
            // quantization noise that would cause graininess at certain frequencies
            phaseAccumulator[voiceIndex] += accumulatorAdd;

            // Wrap around at 24-bit boundary (0x1000000)
            if (phaseAccumulator[voiceIndex] > 0xFFFFFF)
                phaseAccumulator[voiceIndex] -= 0x1000000;
        }

        // Detect MSB transitions for hard sync
        // The MSB is bit 23 (0x800000)
        uint32_t MSB = static_cast<uint32_t>(phaseAccumulator[voiceIndex]) & 0x800000;
        sourceMSBrise[voiceIndex] = (MSB > (static_cast<uint32_t>(previousAccumulator[voiceIndex]) & 0x800000)) ? 1 : 0;

        uint16_t waveformOutput = 0;  // Waveform output (16-bit unsigned)

        // =====================================================================
        // WAVEFORM GENERATION
        // =====================================================================
        // The SID has 4 basic waveforms that can be combined. The waveform
        // selection bits are in the control register's upper nibble.

        if (waveformCTRL & NOISE_BITMASK)
        {
            // -------------------------------------------------------------------
            // NOISE WAVEFORM
            // -------------------------------------------------------------------
            // Noise is generated by a 23-bit Linear Feedback Shift Register (LFSR)
            // The LFSR is clocked when bit 19 of the phase accumulator transitions
            // The output is 8 specific bits from the LFSR, creating a pseudo-random
            // sequence that repeats every 8,388,607 samples (for maximum-length LFSR)

            tmp = noiseLFSR[voiceIndex];

            // Clock the LFSR when bit 19 transitions or when frequency is very high
            // Bit 19 (0x100000) serves as the clock input to the noise generator
            if (((static_cast<uint32_t>(phaseAccumulator[voiceIndex]) & 0x100000) !=
                 (static_cast<uint32_t>(previousAccumulator[voiceIndex]) & 0x100000)) ||
                accumulatorAdd >= 0x100000)
            {
                // LFSR feedback polynomial: bits 22 and 17
                // XOR these bits and shift them in at the LSB
                uint32_t step = (tmp & 0x400000) ^ ((tmp & 0x20000) << 5);
                tmp = ((tmp << 1) + (step > 0 || test)) & 0x7FFFFF;
                noiseLFSR[voiceIndex] = tmp;
            }

            // Extract noise output from 8 specific LFSR bits
            // These bits were chosen to give good spectral characteristics
            if (waveformCTRL & 0x70)
            {
                // Noise combined with other waveforms: output zero
                // This matches real SID behavior (noise kills other waveforms)
                waveformOutput = 0;
            }
            else
            {
                // Pure noise: extract and recombine 8 bits from the LFSR
                // Bit pattern: 20,18,14,11,9,5,2,0 shifted to form 8-bit value
                waveformOutput = ((tmp & 0x100000) >> 5) + ((tmp & 0x40000) >> 4) +
                       ((tmp & 0x4000) >> 1) + ((tmp & 0x800) << 1) +
                       ((tmp & 0x200) << 2) + ((tmp & 0x20) << 5) +
                       ((tmp & 0x04) << 7) + ((tmp & 0x01) << 8);
            }
        }
        else if (waveformCTRL & PULSE_BITMASK)
        {
            // -------------------------------------------------------------------
            // PULSE WAVEFORM
            // -------------------------------------------------------------------
            // Pulse is a rectangular wave with variable duty cycle (pulse width)
            // Pulse width is 12-bit (registers 2-3), scaled up to 16-bit range

            // Read pulse width and scale to 16-bit range (* 16)
            uint16_t pulseWidth = (voiceRegister[2] | ((voiceRegister[3] & 0xF) << 8)) << 4;

            // Calculate step size for band-limiting (anti-aliasing)
            // This is used to smooth transitions at pulse edges
            tmp = static_cast<int>(accumulatorAdd) >> 9;

            // Limit pulse width to valid range
            // PW must be between tmp and (0xFFFF - tmp) to avoid artifacts
            if (0 < pulseWidth && pulseWidth < tmp) pulseWidth = tmp;
            tmp ^= 0xFFFF;
            if (pulseWidth > tmp) pulseWidth = tmp;

            // Get current phase (16-bit, upper bits of 24-bit accumulator)
            tmp = static_cast<uint32_t>(phaseAccumulator[voiceIndex]) >> 8;

            if (waveformCTRL == PULSE_BITMASK)
            {
                // -------------------------------------------------------------------
                // PURE PULSE WAVEFORM (no other waveforms combined)
                // -------------------------------------------------------------------
                // Use band-limited synthesis to reduce aliasing at pulse edges
                // We use floating-point arithmetic to avoid quantization artifacts
                // that caused graininess at certain frequencies with integer math

                // Calculate band-limiting step (how many samples for the transition)
                double step = 256.0 / (static_cast<int>(accumulatorAdd) >> 16);
                if (step == 0) step = 1.0;  // Avoid division by zero at very high frequencies

                if (test)
                {
                    // Test bit set: output maximum (all bits high)
                    waveformOutput = 0xFFFF;
                }
                else if (tmp < pulseWidth)
                {
                    // Rising edge of pulse (phase crossing pulse width threshold)
                    // Apply band-limiting to smooth the transition from low to high
                    double lim = (0xFFFF - pulseWidth) * step;
                    if (lim > 0xFFFF) lim = 0xFFFF;

                    // Linear interpolation across the transition
                    double waveformOutput_d = lim - (pulseWidth - tmp) * step;
                    if (waveformOutput_d < 0) waveformOutput_d = 0;

                    waveformOutput = static_cast<uint16_t>(waveformOutput_d);
                }
                else
                {
                    // Falling edge of pulse (phase wrapping around)
                    // Apply band-limiting to smooth the transition from high to low
                    double lim = pulseWidth * step;
                    if (lim > 0xFFFF) lim = 0xFFFF;

                    // Linear interpolation across the transition
                    double waveformOutput_d = (0xFFFF - tmp) * step - lim;
                    if (waveformOutput_d >= 0) waveformOutput_d = 0xFFFF;

                    waveformOutput = static_cast<uint16_t>(waveformOutput_d) & 0xFFFF;
                }
            }
            else
            {
                // -------------------------------------------------------------------
                // COMBINED PULSE WAVEFORMS
                // -------------------------------------------------------------------
                // Pulse combined with triangle and/or sawtooth
                // Use lookup tables for the complex analog interactions

                // Basic pulse: 0xFFFF if phase >= pulseWidth, else 0
                waveformOutput = (tmp >= pulseWidth || test) ? 0xFFFF : 0;

                if (waveformCTRL & TRI_BITMASK)
                {
                    if (waveformCTRL & SAW_BITMASK)
                    {
                        // Pulse + Triangle + Sawtooth: all three waveforms
                        waveformOutput = waveformOutput ? reSIDuEngine::SID::combinedWF(voiceIndex, pulseTriSaw8580, tmp >> 4, true) : 0;
                    }
                    else
                    {
                        // Pulse + Triangle: ring modulation affects the triangle
                        // XOR phase with previous voice's MSB if ring mod is enabled
                        // For ring mod: voice 0 modulated by voice 2, voice 1 by voice 0, voice 2 by voice 1
                        int ringSource = (voiceIndex + 2) % SID_CHANNELS;
                        tmp = static_cast<uint32_t>(phaseAccumulator[voiceIndex]) ^
                              (ctrl & RING_BITMASK ? sourceMSB[ringSource] : 0);

                        // Triangle waveform: fold phase at midpoint to create triangle shape
                        // Then use pulse+triangle combined waveform table
                        waveformOutput = waveformOutput ? combinedWF(voiceIndex, pulseTriangle8580,
                                       (tmp ^ (tmp & 0x800000 ? 0xFFFFFF : 0)) >> 11, false) : 0;
                    }
                }
                else if (waveformCTRL & SAW_BITMASK)
                {
                    // Pulse + Sawtooth
waveformOutput = waveformOutput ? reSIDuEngine::SID::combinedWF(voiceIndex, pulseSaw8580,
                            tmp >> 4, true) : 0;
                }
            }
        }
        else if (waveformCTRL & SAW_BITMASK)
        {
            // -------------------------------------------------------------------
            // SAWTOOTH WAVEFORM
            // -------------------------------------------------------------------
            // Sawtooth is a linear ramp from 0 to 0xFFFF
            // It's simply the upper 16 bits of the 24-bit phase accumulator

            waveformOutput = static_cast<uint32_t>(phaseAccumulator[voiceIndex]) >> 8;

            if (waveformCTRL & TRI_BITMASK)
            {
                // Sawtooth + Triangle combined waveform
                // Use lookup table for analog interaction
                waveformOutput = combinedWF(voiceIndex, triSaw8580, waveformOutput >> 4, true);
            }
            else
            {
                // -------------------------------------------------------------------
                // PURE SAWTOOTH WAVEFORM
                // -------------------------------------------------------------------
                // Apply band-limiting to reduce aliasing at the discontinuity
                // The saw has a sharp drop from maximum to minimum once per cycle
                // We use floating-point arithmetic to avoid quantization artifacts

                double step = accumulatorAdd / 0x1200000;  // Normalized step size
                double waveformOutput_d = waveformOutput + waveformOutput * step;

                // Near the discontinuity, apply correction to smooth the transition
                if (waveformOutput_d > 0xFFFF)
                    waveformOutput_d = 0xFFFF - (waveformOutput_d - 0x10000) / step;

                waveformOutput = static_cast<uint16_t>(waveformOutput_d);
            }
        }
        else if (waveformCTRL & TRI_BITMASK)
        {
            // -------------------------------------------------------------------
            // TRIANGLE WAVEFORM
            // -------------------------------------------------------------------
            // Triangle is created by folding the phase accumulator at its midpoint
            // First half: ramp up (0 to max)
            // Second half: ramp down (max to 0)
            // Ring modulation XORs the phase with the previous voice's MSB
            // For ring mod: voice 0 modulated by voice 2, voice 1 by voice 0, voice 2 by voice 1
            int ringSource = (voiceIndex + 2) % SID_CHANNELS;

            // Apply ring modulation if enabled
            tmp = static_cast<uint32_t>(phaseAccumulator[voiceIndex]) ^
                  (ctrl & RING_BITMASK ? sourceMSB[ringSource] : 0);

            // Create triangle by folding: if MSB is set, invert all bits
            // Then shift right by 7 to get 16-bit output (17-bit intermediate)
            waveformOutput = (tmp ^ (tmp & 0x800000 ? 0xFFFFFF : 0)) >> 7;
        }

        // -------------------------------------------------------------------
        // WAVEFORM 00 (no waveform bits set)
        // -------------------------------------------------------------------
        // When no waveform is selected, the DAC holds its last value (floating)
        // This is used to create smooth transitions between waveforms
        if (waveformCTRL)
            previousWFOut[voiceIndex] = waveformOutput;  // Update held value
        else
            waveformOutput = previousWFOut[voiceIndex];  // Use previously held value

        // Store current phase for next sample's calculations
        previousAccumulator[voiceIndex] = phaseAccumulator[voiceIndex];

        // Store MSB for ring modulation and sync of other voices
        sourceMSB[voiceIndex] = MSB;

        // =====================================================================
        // SIGNAL ROUTING AND MIXING
        // =====================================================================
        // Each voice can be routed either to the filter or directly to output
        // The routing is controlled by the filter routing register (0xD417)

        // Check if this voice is muted (reSIDfp API compatibility)
        if (!voiceMuted[voiceIndex])
        {
            // Convert waveform from unsigned (0-65535) to signed (-32768 to +32767)
            // Then scale by envelope (0-255) to apply amplitude control
            if (SIDRegister[0x17] & FILTSW[voiceIndex])
            {
                // Route to filter input
                filterInput += (waveformOutput - 0x8000) * (envelopeCounter[voiceIndex] / 256.0);
            }
            else if (voiceIndex != 2 || !(SIDRegister[0x18] & OFF3_BITMASK))
            {
                // Route directly to output (bypassing filter)
                // Exception: Voice 3 can be muted with the OFF3 bit
                output += (waveformOutput - 0x8000) * (envelopeCounter[voiceIndex] / 256.0);
            }
        }
    }

    // =========================================================================
    // UPDATE READABLE REGISTERS
    // =========================================================================
    // Voice 3's oscillator and envelope can be read back for various purposes
    // (random number generation, music synchronization, etc.)

    // Update oscillator 3 register (upper 8 bits of waveform output)
    SIDRegister[0x1B] = previousWFOut[2] >> 8;

    // Update envelope 3 register (current ADSR level)
    SIDRegister[0x1C] = envelopeCounter[2];

    // =========================================================================
    // EXTERNAL AUDIO INPUT (EXT IN)
    // =========================================================================
    // The SID chip has an external audio input that can be mixed with the
    // SID's output or routed through the filter. The EXTIN bit in register 0x17
    // controls whether EXT IN is routed to the filter.

    if (externalInput != 0)
    {
        if (SIDRegister[0x17] & EXTIN_BITMASK)
        {
            // EXT IN routed to filter
            filterInput += externalInput;
        }
        else
        {
            // EXT IN routed directly to output
            output += externalInput;
        }
        // Consume the external input sample (single-use)
        externalInput = 0;
    }

    // =========================================================================
    // BI-QUADRATIC FILTER
    // =========================================================================
    // The SID's filter is a state-variable (two-integrator-loop) topology
    // that can simultaneously produce lowpass, bandpass, and highpass outputs
    // The user selects which mode(s) to mix into the final output

    // Check if filter is enabled (reSIDfp API compatibility)
    if (filterEnabled)
    {
        // Calculate filter cutoff frequency from 11-bit cutoff register
        // Cutoff is stored in registers 0x15 (lower 3 bits) and 0x16 (upper 8 bits)
        double cutoff = (SIDRegister[0x15] & 7) / 8.0 + SIDRegister[0x16] + 0.2;

        double resonance;  // Filter resonance (Q factor)

        if (sidModel == MOS8580)
        {
            // 8580 filter: more linear, better behaved
            // Cutoff range approximately 0-12.5 kHz
            cutoff = 1.0 - std::exp(cutoff * cutoffRatio8580);

            // Resonance from 4-bit value (upper nibble of register 0x17)
            // Maps approximately to Q from 0.707 to 8.0
            resonance = std::pow(2.0, (4.0 - (SIDRegister[0x17] >> 4)) / 8.0);
        }
        else
        {
            // 6581 filter: more quirky, with non-linear low end
            // Cutoff range approximately 0-20 kHz but with dead zone at low settings
            if (cutoff < 24)
                cutoff = 0.035;  // Dead zone: very low cutoff values produce minimal filtering
            else
                cutoff = 1.0 - 1.263 * std::exp(cutoff * cutoffRatio6581);

            // Resonance behavior is different on 6581
            // High resonance values can cause filter oscillation
            resonance = (SIDRegister[0x17] > 0x5F) ?
                        8.0 / (SIDRegister[0x17] >> 4) : 1.41;
        }

        // Two-integrator-loop filter implementation
        // This creates a state-variable filter with simultaneous LP/BP/HP outputs

        // Resonance gain compensation: High Q (low resonance coefficient) causes
        // gain boost at the cutoff frequency. Compensate by scaling outputs by
        // the resonance coefficient (which is essentially 1/Q).
        // Clamp to prevent complete muting at very high resonance settings.
        double resonanceCompensation = std::max(0.5, resonance);

        // Highpass output: input - (resonance * BP + LP)
        // This is the input signal with low and mid frequencies removed
        double tmp = filterInput + previousBandpass * resonance + previousLowpass;
        if (SIDRegister[0x18] & HIGHPASS_BITMASK)
            output -= tmp * resonanceCompensation;  // Mix highpass into output if enabled

        // Update bandpass integrator: BP = BP - (HP * cutoff)
        // Bandpass removes highs and lows, leaving only mid frequencies
        tmp = previousBandpass - tmp * cutoff;
        previousBandpass = tmp;
        if (SIDRegister[0x18] & BANDPASS_BITMASK)
            output -= tmp * resonanceCompensation;  // Mix bandpass into output if enabled

        // Update lowpass integrator: LP = LP + (BP * cutoff)
        // Lowpass removes high frequencies, leaving only lows
        tmp = previousLowpass + tmp * cutoff;
        previousLowpass = tmp;
        if (SIDRegister[0x18] & LOWPASS_BITMASK)
            output += tmp * resonanceCompensation;  // Mix lowpass into output if enabled
    }
    else
    {
        // Filter disabled: route filter input directly to output
        output += filterInput;
    }

    // =========================================================================
    // MASTER VOLUME AND OUTPUT SCALING
    // =========================================================================
    // Apply master volume (4-bit, lower nibble of 0xD418)
    // Scale output by the maximum amplitude of a single voice (0x10000)
    //
    // With 3 voices at maximum envelope (255) and waveform (±32768):
    // Max per voice = 32768 * (255/256) ≈ 32640
    // Three voices can theoretically reach ±97920 before volume scaling
    // After dividing by 0x10000 (65536), range is approximately ±1.5
    // The /15.0 from master volume brings it to approximately ±0.1 to ±1.5
    // depending on the volume setting (0-15)

    return static_cast<float>((output / 0x10000) * (SIDRegister[0x18] & 0xF) / 15.0);
}

/**
 * Clock the SID chip and generate audio samples (reSIDfp API compatible).
 *
 * Advances the SID emulation by the specified number of CPU cycles and generates
 * the corresponding audio samples into the provided buffer. The number of samples
 * generated is calculated based on the clock ratio (CPU cycles per sample).
 *
 * @param cycles Number of CPU cycles to advance the emulation
 * @param buf Buffer to receive 16-bit signed audio samples, or NULL to discard output
 * @return Number of audio samples generated
 */
int SID::clock(unsigned int cycles, short *buf)
{
    // Calculate how many samples to generate based on CPU cycles
    // clkRatio = CPU cycles per sample, so samples = cycles / clkRatio
    int samples = static_cast<int>(cycles / clkRatio);

    if (buf == nullptr)
    {
        // NULL buffer: generate samples but don't store (same as clockSilent)
        for (int i = 0; i < samples; i++)
        {
            processSID();
        }
    }
    else
    {
        // Generate samples and store in buffer
        for (int i = 0; i < samples; i++)
        {
            // Process SID and apply master volume
            float sample = processSID() * volume;

            // Convert from float to 16-bit signed integer
            // Clamp to prevent overflow
            if (sample > 1.0f)
                sample = 1.0f;
            else if (sample < -1.0f)
                sample = -1.0f;

            // Scale to 16-bit range and store
            buf[i] = static_cast<short>(sample * 32767.0f);
        }
    }

    return samples;
}

/**
 * Generate one audio sample (convenience method).
 *
 * This is a convenience wrapper for single-sample generation. Call this function
 * once per output sample at your desired sample rate. It processes the SID chip
 * and returns the audio output as a float.
 *
 * @return Audio output from the SID chip, scaled by master volume
 */
float SID::clockSample()
{
    // Process the SID chip and apply master volume
    return processSID() * volume;
}

// ============================================================================
// reSIDfp API Compatibility Methods
// ============================================================================

/**
 * Set sampling parameters (reSIDfp API compatibility).
 *
 * Configures the clock frequency and sample rate for the SID emulation.
 * The sampling method and highest accurate frequency parameters are accepted
 * for API compatibility but are not used in this implementation, as reSIDuEngine
 * always uses per-sample processing.
 *
 * @param clockFrequency CPU clock frequency in Hz (985248 for PAL, 1022730 for NTSC)
 * @param method Sampling method (ignored - kept for API compatibility)
 * @param samplingFrequency Audio output sample rate in Hz (e.g., 44100)
 * @param highestAccurateFrequency Passband frequency (ignored - kept for API compatibility)
 */
void SID::setSamplingParameters(double clockFrequency, int /*method*/,
                               double samplingFrequency, double /*highestAccurateFrequency*/)
{
    // Update the CPU clock frequency
    cpuClock = clockFrequency;

    // Update the sample rate
    sampleRate = samplingFrequency;

    // Recalculate clock ratio and timing
    updateClockRatio();

    // Recalculate filter coefficients for the new sample rate
    cutoffRatio8580 = -2.0 * M_PI * (12500.0 / 256.0) / sampleRate;
    cutoffRatio6581 = -2.0 * M_PI * (20000.0 / 256.0) / sampleRate;

    // Note: method and highestAccurateFrequency parameters are ignored
    // reSIDuEngine always uses sample-accurate processing
}

/**
 * Clock the SID chip silently without generating audio output.
 *
 * This method advances the SID emulation state (envelopes, oscillators, etc.)
 * by the specified number of CPU cycles without actually computing audio samples.
 * This is useful for fast-forwarding or seeking in SID playback.
 *
 * The implementation approximates the state advancement by calculating how many
 * audio samples would correspond to the given number of cycles and then clocking
 * the SID that many times (but discarding the output).
 *
 * @param cycles Number of CPU cycles to advance
 */
void SID::clockSilent(unsigned int cycles)
{
    // Calculate how many samples correspond to this many CPU cycles
    // clkRatio = CPU cycles per sample, so samples = cycles / clkRatio
    unsigned int samples = static_cast<unsigned int>(cycles / clkRatio);

    // Clock the SID for that many samples, discarding the output
    for (unsigned int i = 0; i < samples; i++)
    {
        processSID();
    }
}

/**
 * Enable or disable filter emulation.
 *
 * When the filter is disabled, all voices bypass the filter and go directly
 * to the output mixer. This can be useful for debugging, performance testing,
 * or comparing filtered vs. unfiltered sound.
 *
 * @param enable True to enable filter emulation, false to disable
 */
void SID::enableFilter(bool enable)
{
    filterEnabled = enable;
}

/**
 * Mute or unmute a specific voice.
 *
 * Allows individual voices to be silenced without affecting their internal
 * state. The oscillators and envelopes continue running normally, but the
 * voice's output is not mixed into the final audio.
 *
 * @param channel Voice number (0-2)
 * @param enable True to mute the voice, false to unmute
 */
void SID::mute(int channel, bool enable)
{
    if (channel >= 0 && channel < SID_CHANNELS)
    {
        voiceMuted[channel] = enable;
    }
}

/**
 * Input external audio signal (EXT IN).
 *
 * The SID chip has an external audio input (EXT IN) that can be mixed with
 * the SID's own output or routed through the filter. This method stores a
 * 16-bit signed audio sample that will be mixed into the next audio sample
 * generated by processSID().
 *
 * The external input can be routed to the filter by setting the EXT bit
 * in the filter routing register (0xD417 bit 3). If not routed to the filter,
 * it is mixed directly with the output.
 *
 * @param value 16-bit signed audio sample to inject (-32768 to +32767)
 */
void SID::input(int value)
{
    externalInput = value;
}

} // namespace reSIDuEngine
