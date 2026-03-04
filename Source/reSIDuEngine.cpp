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
#include <vector>

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
 * Hardware-accurate ADSR rate comparison values for the 15-bit LFSR rate counter.
 *
 * The SID's envelope generator uses a 15-bit LFSR (linear feedback shift register)
 * clocked once per CPU cycle. When the LFSR matches the rate-specific comparison
 * value the envelope counter is stepped and the LFSR resets to 0x7fff.
 * The period for each rate is the number of LFSR clocks from 0x7fff to the match.
 *
 * Source: kevtris.org SID ADSR hardware analysis
 */
static constexpr uint16_t adsrtable[16] = {
    0x007f, 0x3000, 0x1e00, 0x0660, 0x0182, 0x5573, 0x000e, 0x3805,
    0x2424, 0x2220, 0x090c, 0x0ecd, 0x010e, 0x23f7, 0x5237, 0x64a8
};

/**
 * Simulate the 15-bit LFSR and return the number of clocks from 0x7fff until
 * lfsr matches the target comparison value.  This is the envelope rate period
 * in CPU clock cycles for the corresponding ADSR rate register setting.
 *
 * LFSR feedback: new_bit = bit14 XOR bit13 (tapped from the two MSBs).
 */
static uint32_t computeLFSRPeriod(uint16_t target)
{
    uint16_t lfsr = 0x7fff;
    uint32_t count = 0;
    do {
        const uint16_t feedback = ((lfsr << 14) ^ (lfsr << 13)) & 0x4000;
        lfsr = (lfsr >> 1) | feedback;
        ++count;
    } while (lfsr != target);
    return count;
}

/**
 * Build a normalized R-2R ladder DAC lookup table.
 * out[i] is in [0,1] representing the analog output for digital input i.
 * rRatio: 2R/R ratio (2.20 for MOS6581, 2.00 for MOS8580)
 * hasTerm: whether bit 0 has a termination resistor (false for MOS6581)
 * leakage: sub-threshold MOSFET leakage fraction (0.0075 / 0.0035)
 */
static void buildKinkedDacTable(double* out, int bits,
                                double rRatio, bool hasTerm, double leakage)
{
    constexpr double R_INFINITY = 1e6;
    const double _2R = rRatio;       // normalized R = 1.0
    std::vector<double> dac(bits);
    double Vsum = 0.0;

    for (int set_bit = 0; set_bit < bits; set_bit++)
    {
        double Vn = 1.0;
        double R  = 1.0;
        double Rn = hasTerm ? _2R : R_INFINITY;

        // Build "tail" resistance by repeated parallel substitution
        for (int bit = 0; bit < set_bit; bit++)
            Rn = (Rn == R_INFINITY) ? R + _2R : R + (_2R * Rn) / (_2R + Rn);

        // Source transformation for bit voltage
        if (Rn == R_INFINITY) {
            Rn = _2R;
        } else {
            Rn = (_2R * Rn) / (_2R + Rn);
            Vn = Vn * Rn / _2R;
        }

        // Propagate to output
        for (int bit = set_bit + 1; bit < bits; bit++) {
            Rn += R;
            const double I = Vn / Rn;
            Rn = (_2R * Rn) / (_2R + Rn);
            Vn = Rn * I;
        }

        dac[set_bit] = Vn;
        Vsum += Vn;
    }

    // Normalize and accumulate into output table for each input value
    for (int i = 0; i < bits; i++)
        dac[i] /= Vsum;

    const int tableSize = 1 << bits;
    for (int input = 0; input < tableSize; input++)
    {
        double val = 0.0;
        for (int i = 0; i < bits; i++)
            val += ((input >> i) & 1) ? dac[i] : dac[i] * leakage;
        out[input] = val;
    }
}

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
    , externalLowpass(0.0f)
    , externalHighpass(0.0f)
{
    // Initialize voice mute flags (all voices unmuted by default)
    voiceMuted.fill(false);
    // Zero-initialize all 32 SID registers
    SIDRegister.fill(0);
    // Zero-initialize fractional accumulator carry
    phaseAccumulatorFrac.fill(0.0);

    // Calculate the clock ratio: how many CPU cycles occur per audio sample
    // This is critical for pitch accuracy - it determines phase accumulator advancement
    updateClockRatio();

    // Build hardware-accurate FC cutoff coefficient tables via kinked DAC simulation.
    // The 11-bit FC register indexes into an R-2R ladder DAC; DAC output (normalized
    // 0-1) is used to drive the exponential filter coefficient formula.
    // 6581: 2R/R=2.20, missing bit-0 termination (creates dead zone at low FC)
    // 8580: 2R/R=2.00, proper termination (nearly linear)
    {
        std::vector<double> dac6581(2048), dac8580(2048);
        buildKinkedDacTable(dac6581.data(), 11, 2.20, false, 0.0075);
        buildKinkedDacTable(dac8580.data(), 11, 2.00, true,  0.0035);

        // Max cutoff Hz per chip model (controls the frequency at FC=2047).
        // 6581: 16 kHz matches hardware-measured upper bound and calibrates the
        //   exponential coefficient to produce the same mid-range cutoff as the
        //   old formula (within ~1% at FC_HI=100).
        // 8580: 12.5 kHz is identical to the old formula's effective maximum.
        const double maxHz6581 = 16000.0;
        const double maxHz8580 = 12500.0;
        const double ratio6581 = -2.0 * M_PI * maxHz6581 / sampleRate;
        const double ratio8580 = -2.0 * M_PI * maxHz8580 / sampleRate;

        for (int fc = 0; fc < 2048; fc++) {
            fcCutoffTable6581[fc] = static_cast<float>(1.0 - std::exp(dac6581[fc] * ratio6581));
            fcCutoffTable8580[fc] = static_cast<float>(1.0 - std::exp(dac8580[fc] * ratio8580));
        }

        // The 6581 R-2R DAC is non-monotonic: bits 0-2 have similar weights due to the
        // missing bit-0 termination, so fc=7→8 (FC_LO roll-over) briefly drops the
        // cutoff. Real hardware smooths this analogically; our digital SVF cannot, so
        // we enforce monotonicity to prevent audible bumps in filter sweeps.
        float prev = 0.0f;
        for (int fc = 0; fc < 2048; fc++) {
            fcCutoffTable6581[fc] = std::max(fcCutoffTable6581[fc], prev);
            prev = fcCutoffTable6581[fc];
        }
    }

    // External RC filter coefficients (models C64 audio output stage):
    //   LP: R=10 kΩ, C=1000 pF → ~16 kHz  (near-transparent at audio rates)
    //   HP: R=10 kΩ, C=10 µF  → ~1.6 Hz   (removes DC offset from filter integrators)
    {
        const double dt = 1.0 / sampleRate;
        w0lp = static_cast<float>(dt / (dt + 10e3 * 1000e-12));
        w0hp = static_cast<float>(dt / (dt + 10e3 * 10e-6));
    }

    // Build oscillator waveform DAC tables (12-bit, 4096 entries per chip model).
    // waveformOutput is 16-bit; upper 12 bits (>> 4) index these tables.
    // Values are centered at the DAC midpoint and scaled to match the existing
    // signal range (~[-32768, +32767]), so only the nonlinear shape changes.
    {
        std::vector<double> raw(4096);

        // 6581: kinked R-2R + cubic NMOS source-follower saturation model.
        // Saturation: V' = GAIN*V + (1-GAIN)*SAT*V^3  (residfpII Dac::getOutput, saturate=true)
        buildKinkedDacTable(raw.data(), 12, 2.20, false, 0.0075);
        {
            constexpr double GAIN = 1.1, SAT = 1.1;
            auto saturate = [&](double v) {
                return GAIN * v + (1.0 - GAIN) * SAT * v * v * v;
            };
            const double offset = saturate(raw[0x7FF]);
            for (int i = 0; i < 4096; i++)
                oscDAC6581[i] = static_cast<float>((saturate(raw[i]) - offset) * 65535.0);
        }

        // 8580: ideal kinked R-2R, no saturation.
        buildKinkedDacTable(raw.data(), 12, 2.00, true, 0.0035);
        {
            const double offset = raw[0x7FF];
            for (int i = 0; i < 4096; i++)
                oscDAC8580[i] = static_cast<float>((raw[i] - offset) * 65535.0);
        }
    }

    // Build envelope DAC tables (8-bit, 256 entries per chip model).
    // Output is normalized to [0, 1]; envDAC[255] == 1.0 matches the prior 255/256.0 max.
    {
        std::vector<double> raw(256);
        buildKinkedDacTable(raw.data(), 8, 2.20, false, 0.0075);  // 6581
        for (int i = 0; i < 256; i++) envDAC6581[i] = static_cast<float>(raw[i]);
        buildKinkedDacTable(raw.data(), 8, 2.00, true,  0.0035);  // 8580
        for (int i = 0; i < 256; i++) envDAC8580[i] = static_cast<float>(raw[i]);
    }

    // Build hardware-accurate ADSR period table from 15-bit LFSR simulation.
    // Each period is the number of CPU clock cycles between envelope steps,
    // derived by counting LFSR clocks from 0x7fff to the rate-specific match value.
    for (int i = 0; i < 16; i++)
        adsrPeriods[i] = static_cast<float>(computeLFSRPeriod(adsrtable[i]));

    // Build the ADSR exponential decay prescaler table.
    // During decay and release the envelope counter is only decremented every N
    // rate-counter firings, where N depends on the current envelope level.
    // This creates the hardware's exponential (logarithmic) decay characteristic.
    // Threshold values and prescaler periods are measured from real 6581/8580 hardware:
    //   env >= 0x5d → every firing   (period 1)
    //   env >= 0x36 → every 2nd      (period 2)
    //   env >= 0x1a → every 4th      (period 4)
    //   env >= 0x0e → every 8th      (period 8)
    //   env >= 0x06 → every 16th     (period 16)
    //   env <  0x06 → every 30th     (period 30) – slowest, near silence
    for (int i = 0; i < 256; i++)
    {
        if      (i < 0x06) adsrExptable[i] = 30;
        else if (i < 0x0e) adsrExptable[i] = 16;
        else if (i < 0x1a) adsrExptable[i] = 8;
        else if (i < 0x36) adsrExptable[i] = 4;
        else if (i < 0x5d) adsrExptable[i] = 2;
        else               adsrExptable[i] = 1;
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

    // Pulse + Triangle: moderate interaction, medium threshold (based on SID chip measurements)
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
        phaseAccumulatorFrac[i] = 0.0; // Fractional carry

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
    externalLowpass  = 0.0f; // External RC lowpass state
    externalHighpass = 0.0f; // External RC highpass state

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
    int reg = addr & 0x1F;

    // Detect gate transitions immediately on control register writes.
    // Control registers: offset 4 (voice 0), 11 (voice 1), 18 (voice 2).
    // This ensures gate-off/gate-on sequences within a single sample
    // period are not lost (the ADSR state machine sees both transitions).
    if (reg == 4 || reg == 11 || reg == 18)
    {
        int voiceIndex = (reg - 4) / 7;
        uint8_t previousGate = adsrState[voiceIndex] & GATE_BITMASK;
        uint8_t newGate = value & GATE_BITMASK;

        if (previousGate != newGate)
        {
            if (previousGate)
            {
                // Gate falling edge (note-off): enter Release phase
                adsrState[voiceIndex] &= 0xFF - (GATE_BITMASK | ATTACK_BITMASK | DECAYSUSTAIN_BITMASK);
            }
            else
            {
                // Gate rising edge (note-on): enter Attack phase
                adsrState[voiceIndex] = GATE_BITMASK | ATTACK_BITMASK | DECAYSUSTAIN_BITMASK;
            }
        }
    }

    SIDRegister[reg] = value;
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
 * Get measured combined waveform value using frequency-dependent filtering approach.
 *
 * This function implements advanced combined waveform processing,
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
    
    // Apply frequency-dependent filtering using 0x7777 + 0x8888/pitch coefficients
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

    // Select chip-model-specific DAC tables for this sample.
    const float* oscDAC = (sidModel == MOS6581) ? oscDAC6581.data() : oscDAC8580.data();
    const float* envDAC = (sidModel == MOS6581) ? envDAC6581.data() : envDAC8580.data();

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

        // Advance the rate counter by the number of CPU cycles elapsed this sample.
        // The counter is in CPU-cycle units; adsrPeriods[] holds the hardware-accurate
        // cycle count for each rate (derived from the 15-bit LFSR simulation).
        rateCounter[voiceIndex] += clkRatio;

        // ADSR delay bug emulation: the real SID's 15-bit rate counter wraps at 0x8000.
        // If a new (shorter) rate is written while the counter is already past its
        // comparison value, the counter wraps around before the next firing.
        if (rateCounter[voiceIndex] >= 0x8000)
            rateCounter[voiceIndex] -= 0x8000;

        // Determine which ADSR phase we're in and select the period for that rate.
        float period;
        int rateIndex;

        if (adsrState[voiceIndex] & ATTACK_BITMASK)
        {
            rateIndex = voiceRegister[5] >> 4;        // Attack rate (upper nibble of AD)
            period    = adsrPeriods[rateIndex];
        }
        else if (adsrState[voiceIndex] & DECAYSUSTAIN_BITMASK)
        {
            rateIndex = voiceRegister[5] & 0xF;       // Decay rate (lower nibble of AD)
            period    = adsrPeriods[rateIndex];
        }
        else
        {
            rateIndex = SR & 0xF;                     // Release rate (lower nibble of SR)
            period    = adsrPeriods[rateIndex];
        }
        (void)rateIndex;  // only used for period lookup above

        // Fire the envelope step once for each full period that has elapsed.
        // Using a while loop (rather than a single if) handles fast rates where the
        // LFSR period is shorter than one audio sample (e.g. rate 0 fires ~3×/sample).
        while (rateCounter[voiceIndex] >= period && tmp == 0)
        {
            rateCounter[voiceIndex] -= period;

            // During Attack the envelope always steps every firing.
            // During Decay/Release it is gated by the exponential prescaler,
            // which slows the decrement as the envelope approaches silence.
            if ((adsrState[voiceIndex] & ATTACK_BITMASK) ||
                ++exponentCounter[voiceIndex] == adsrExptable[envelopeCounter[voiceIndex]])
            {
                if (!(adsrState[voiceIndex] & HOLDZERO_BITMASK))
                {
                    if (adsrState[voiceIndex] & ATTACK_BITMASK)
                    {
                        // Attack: increment toward maximum (255), then switch to Decay.
                        ++envelopeCounter[voiceIndex];
                        if (envelopeCounter[voiceIndex] >= 0xFF)
                        {
                            envelopeCounter[voiceIndex] = 0xFF;
                            adsrState[voiceIndex] &= 0xFF - ATTACK_BITMASK;
                            break;  // period changes on next sample (decay rate)
                        }
                    }
                    else if (!(adsrState[voiceIndex] & DECAYSUSTAIN_BITMASK) ||
                             envelopeCounter[voiceIndex] > (SR >> 4) + (SR & 0xF0))
                    {
                        // Decay/Release: decrement; hold at zero when floor is reached.
                        --envelopeCounter[voiceIndex];
                        if (envelopeCounter[voiceIndex] <= 0)
                        {
                            envelopeCounter[voiceIndex] = 0;
                            adsrState[voiceIndex] |= HOLDZERO_BITMASK;
                            break;  // counter frozen, no further updates needed
                        }
                    }
                }

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
            // Normal operation: advance phase accumulator.
            // accumulatorAdd (freq * clkRatio) is non-integer, so we accumulate
            // the fractional part separately and carry whole increments into
            // the 24-bit integer accumulator each sample.
            phaseAccumulatorFrac[voiceIndex] += accumulatorAdd;
            uint32_t intAdd = static_cast<uint32_t>(phaseAccumulatorFrac[voiceIndex]);
            phaseAccumulatorFrac[voiceIndex] -= intAdd;
            phaseAccumulator[voiceIndex] = (phaseAccumulator[voiceIndex] + intAdd) & 0xFFFFFF;
        }

        // Detect MSB rising edge (bit 23: 0→1) for hard sync.
        // (~old & new) isolates bits that were 0 in the previous sample and are 1 now.
        uint32_t MSB = phaseAccumulator[voiceIndex] & 0x800000;
        sourceMSBrise[voiceIndex] = ((~previousAccumulator[voiceIndex] & phaseAccumulator[voiceIndex]) & 0x800000) ? 1 : 0;

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
            if (((phaseAccumulator[voiceIndex] & 0x100000) !=
                 (previousAccumulator[voiceIndex] & 0x100000)) ||
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
            tmp = phaseAccumulator[voiceIndex] >> 8;

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
                        tmp = phaseAccumulator[voiceIndex] ^
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

            waveformOutput = phaseAccumulator[voiceIndex] >> 8;

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
            tmp = phaseAccumulator[voiceIndex] ^
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

        // Check if this voice is muted
        if (!voiceMuted[voiceIndex])
        {
            // Apply chip-specific DAC nonlinearity to waveform and envelope.
            // waveformOutput upper 12 bits (>> 4) index the 4096-entry oscDAC table;
            // envelopeCounter (0-255) indexes the 256-entry envDAC table.
            // oscDAC values are centered (midpoint → 0) and scaled to ~[-32768, +32767].
            // envDAC values are normalized to [0, 1].
            const float voiceOut = oscDAC[waveformOutput >> 4] * envDAC[envelopeCounter[voiceIndex]];
            if (SIDRegister[0x17] & FILTSW[voiceIndex])
            {
                // Route to filter input
                filterInput += voiceOut;
            }
            else if (voiceIndex != 2 || !(SIDRegister[0x18] & OFF3_BITMASK))
            {
                // Route directly to output (bypassing filter)
                // Exception: Voice 3 can be muted with the OFF3 bit
                output += voiceOut;
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

    // Check if filter is enabled
    if (filterEnabled)
    {
        // 11-bit FC value from registers: FC_HI (upper 8) | FC_LO (lower 3)
        const int fc = (SIDRegister[0x16] << 3) | (SIDRegister[0x15] & 0x07);
        const double cutoff = (sidModel == MOS8580)
            ? fcCutoffTable8580[fc]
            : fcCutoffTable6581[fc];

        // Resonance feedback gain (4-bit value, upper nibble of RES_FILT register)
        const int res = SIDRegister[0x17] >> 4;
        double resonance;
        if (sidModel == MOS8580) {
            // 8580: exponential approximation of the Rf/Ri resistor-ladder circuit
            resonance = std::pow(2.0, (4.0 - res) / 8.0);
        } else {
            // 6581: feedback gain = (~res & 0xf) / 8.0  (hardware-measured)
            // res=0  → 1.875 (heavily damped), res=15 → 0 (self-oscillates)
            const int inv_res = (~res) & 0xf;
            resonance = inv_res / 8.0;
        }
        // Clamp to prevent SVF divergence. The 6581 hardware self-oscillates at
        // res=15 (resonance=0), but our simple SVF goes unstable at values below
        // ~0.3. Clamping at 0.3 (Q≈3.3) gives audible strong resonance without
        // numerical instability.
        if (resonance < 0.3) resonance = 0.3;

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

    // Scale output
    float result = static_cast<float>((output / 0x10000) * (SIDRegister[0x18] & 0xF) / 15.0f);

    // External RC filter: LP at ~16 kHz (near-transparent), HP at ~1.6 Hz (DC removal)
    externalLowpass  += w0lp * (result      - externalLowpass);
    externalHighpass += w0hp * (externalLowpass - externalHighpass);
    result = externalLowpass - externalHighpass;

    // Soft clipping: tanh-based compression above ±1.0 to prevent harsh digital peaks
    constexpr float CLIP_THRESHOLD = 1.0f;
    constexpr float CLIP_HEADROOM  = 0.5f;
    if (result > CLIP_THRESHOLD)
        result = CLIP_THRESHOLD + CLIP_HEADROOM * std::tanh((result - CLIP_THRESHOLD) / CLIP_HEADROOM);
    else if (result < -CLIP_THRESHOLD)
        result = -CLIP_THRESHOLD - CLIP_HEADROOM * std::tanh((-result - CLIP_THRESHOLD) / CLIP_HEADROOM);

    return result;
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
// Extended API Methods
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

    // Rebuild hardware-accurate FC cutoff coefficient tables for the new sample rate
    {
        std::vector<double> dac6581(2048), dac8580(2048);
        buildKinkedDacTable(dac6581.data(), 11, 2.20, false, 0.0075);
        buildKinkedDacTable(dac8580.data(), 11, 2.00, true,  0.0035);
        const double maxHz6581 = 16000.0;
        const double maxHz8580 = 12500.0;
        const double ratio6581 = -2.0 * M_PI * maxHz6581 / sampleRate;
        const double ratio8580 = -2.0 * M_PI * maxHz8580 / sampleRate;
        for (int fc = 0; fc < 2048; fc++) {
            fcCutoffTable6581[fc] = static_cast<float>(1.0 - std::exp(dac6581[fc] * ratio6581));
            fcCutoffTable8580[fc] = static_cast<float>(1.0 - std::exp(dac8580[fc] * ratio8580));
        }
        float prev = 0.0f;
        for (int fc = 0; fc < 2048; fc++) {
            fcCutoffTable6581[fc] = std::max(fcCutoffTable6581[fc], prev);
            prev = fcCutoffTable6581[fc];
        }
        const double dt = 1.0 / sampleRate;
        w0lp = static_cast<float>(dt / (dt + 10e3 * 1000e-12));
        w0hp = static_cast<float>(dt / (dt + 10e3 * 10e-6));
    }

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
