// reSIDuEngine - C++ implementation of the Commodore 64 SID chip

/*
 * Copyright (c) 2026, Gavin Graham <gavindi@gmail.com>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public License, v. 2.0.
 * If a copy of the MPL was not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

/*
 * Example program demonstrating the reSIDuEngine
 *
 * This creates a simple test tone using the reSIDuEngine
 * and writes it to a WAV file.
 */

#include "reSIDuEngine.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <cstdint>

// Simple WAV file writer
class WavWriter
{
public:
    WavWriter(const std::string& filename, int sampleRate, int numChannels = 1)
        : sampleRate(sampleRate)
        , numChannels(numChannels)
    {
        file.open(filename, std::ios::binary);
        if (!file.is_open())
            throw std::runtime_error("Cannot open file: " + filename);

        // Write placeholder header (will be updated later)
        writeHeader(0);
    }

    ~WavWriter()
    {
        close();
    }

    void writeSample(float sample)
    {
        // Convert float [-1, 1] to 16-bit PCM
        int16_t pcm = static_cast<int16_t>(sample * 32767.0f);
        file.write(reinterpret_cast<char*>(&pcm), sizeof(pcm));
        numSamples++;
    }

    void close()
    {
        if (file.is_open())
        {
            // Update header with actual sample count
            file.seekp(0);
            writeHeader(numSamples);
            file.close();
        }
    }

private:
    void writeHeader(uint32_t samples)
    {
        uint32_t dataSize = samples * numChannels * sizeof(int16_t);
        uint32_t fileSize = dataSize + 36;

        // RIFF header
        file.write("RIFF", 4);
        file.write(reinterpret_cast<char*>(&fileSize), 4);
        file.write("WAVE", 4);

        // fmt chunk
        file.write("fmt ", 4);
        uint32_t fmtSize = 16;
        file.write(reinterpret_cast<char*>(&fmtSize), 4);

        uint16_t audioFormat = 1; // PCM
        file.write(reinterpret_cast<char*>(&audioFormat), 2);

        uint16_t channels = numChannels;
        file.write(reinterpret_cast<char*>(&channels), 2);

        uint32_t rate = sampleRate;
        file.write(reinterpret_cast<char*>(&rate), 4);

        uint32_t byteRate = sampleRate * numChannels * sizeof(int16_t);
        file.write(reinterpret_cast<char*>(&byteRate), 4);

        uint16_t blockAlign = numChannels * sizeof(int16_t);
        file.write(reinterpret_cast<char*>(&blockAlign), 2);

        uint16_t bitsPerSample = 16;
        file.write(reinterpret_cast<char*>(&bitsPerSample), 2);

        // data chunk
        file.write("data", 4);
        file.write(reinterpret_cast<char*>(&dataSize), 4);
    }

    std::ofstream file;
    int sampleRate;
    int numChannels;
    uint32_t numSamples = 0;
};

void setupSawtoothVoice(reSIDuEngine::SID& sid, int voice, uint16_t frequency)
{
    uint16_t baseAddr = 0xD400 + voice * 7;

    // Set frequency
    sid.write(baseAddr + 0, frequency & 0xFF);        // FREQ_LO
    sid.write(baseAddr + 1, (frequency >> 8) & 0xFF); // FREQ_HI

    // Set pulse width (not used for saw, but set anyway)
    sid.write(baseAddr + 2, 0x00); // PW_LO
    sid.write(baseAddr + 3, 0x08); // PW_HI

    // Set ADSR: A=8, D=6, S=0, R=6
    sid.write(baseAddr + 5, 0x86); // ATTACK_DECAY
    sid.write(baseAddr + 6, 0x46); // SUSTAIN_RELEASE

    // Enable sawtooth waveform with gate on
    sid.write(baseAddr + 4, reSIDuEngine::SAW_BITMASK | reSIDuEngine::GATE_BITMASK);
}

void setupPulseVoice(reSIDuEngine::SID& sid, int voice, uint16_t frequency, uint16_t pulseWidth)
{
    uint16_t baseAddr = 0xD400 + voice * 7;

    // Set frequency
    sid.write(baseAddr + 0, frequency & 0xFF);
    sid.write(baseAddr + 1, (frequency >> 8) & 0xFF);

    // Set pulse width
    sid.write(baseAddr + 2, pulseWidth & 0xFF);
    sid.write(baseAddr + 3, (pulseWidth >> 8) & 0xFF);

    // Set ADSR: A=0, D=0, S=15, R=0 (fast attack, full sustain - for testing)
    sid.write(baseAddr + 5, 0x86);
    sid.write(baseAddr + 6, 0x46);

    // Enable pulse waveform with gate on
    sid.write(baseAddr + 4, reSIDuEngine::PULSE_BITMASK | reSIDuEngine::GATE_BITMASK);
}

void setupTriangleVoice(reSIDuEngine::SID& sid, int voice, uint16_t frequency)
{
    uint16_t baseAddr = 0xD400 + voice * 7;

    // Set frequency
    sid.write(baseAddr + 0, frequency & 0xFF);
    sid.write(baseAddr + 1, (frequency >> 8) & 0xFF);

    // Set ADSR: A=0, D=9, S=0, R=0
    sid.write(baseAddr + 5, 0x09);
    sid.write(baseAddr + 6, 0x46);

    // Enable triangle waveform with gate on
    sid.write(baseAddr + 4, reSIDuEngine::TRI_BITMASK | reSIDuEngine::GATE_BITMASK);
}

int main(int argc, char* argv[])
{
    std::cout << "reSIDuEngine Example\n";
    std::cout << "====================\n\n";

    const int SAMPLE_RATE = 44100;
    const double DURATION = 5.0; // seconds
    const int NUM_SAMPLES = static_cast<int>(SAMPLE_RATE * DURATION);

    // Create SID engine
    reSIDuEngine::SID sid(SAMPLE_RATE, reSIDuEngine::MOS8580);
    sid.setVolume(0.5f);

    std::cout << "Initializing SID chip model: 8580\n";
    std::cout << "Sample rate: " << SAMPLE_RATE << " Hz\n";
    std::cout << "Duration: " << DURATION << " seconds\n\n";

    // Setup three voices playing a C major chord
    // C4 = 261.63 Hz, E4 = 329.63 Hz, G4 = 392.00 Hz
    // SID frequency formula: freq_reg = (note_freq * 16777216) / clock_freq
    // With PAL clock = 985248 Hz

    const double CLOCK_FREQ = 985248.0;

    auto calcFreqReg = [CLOCK_FREQ](double noteFreq) -> uint16_t {
        return static_cast<uint16_t>((noteFreq * 16777216.0) / CLOCK_FREQ);
    };

    uint16_t freqC4 = calcFreqReg(261.63);
    uint16_t freqE4 = calcFreqReg(329.63);
    uint16_t freqG4 = calcFreqReg(392.00);

    std::cout << "Programming voices:\n";
    std::cout << "  Voice 1: Sawtooth  @ C4 (" << freqC4 << ")\n";
    std::cout << "  Voice 2: Pulse 50% @ E4 (" << freqE4 << ")\n";
    std::cout << "  Voice 3: Triangle  @ G4 (" << freqG4 << ")\n\n";

    setupSawtoothVoice(sid, 0, freqC4);
    setupPulseVoice(sid, 1, freqE4, 0x800); // 50% pulse width
    setupTriangleVoice(sid, 2, freqG4);

    // Setup filter - DISABLED for testing
    sid.write(0xD415, 0x00); // FC_LO
    sid.write(0xD416, 0x18); // FC_HI - cutoff frequency
    sid.write(0xD417, 0x77); // RES_FILT - NO voices to filter (testing)
    sid.write(0xD418, 0x2F); // MODE_VOL - Low-Pass Filter, volume 15

    std::cout << "Filter: ENABLED (testing)\n\n";

    // Generate audio
    std::cout << "Generating audio...\n";

    try
    {
        WavWriter wav("reSIDuEngine_output.wav", SAMPLE_RATE);

        float minSample = 0.0f, maxSample = 0.0f;

        for (int i = 0; i < NUM_SAMPLES; i++)
        {
            float sample = sid.clockSample();

            // Track min/max for debugging
            if (sample < minSample) minSample = sample;
            if (sample > maxSample) maxSample = sample;

            wav.writeSample(sample);

            // Print progress
            if (i % (SAMPLE_RATE / 10) == 0)
            {
                float progress = 100.0f * i / NUM_SAMPLES;
                std::cout << "\rProgress: " << static_cast<int>(progress) << "%    " << std::flush;
            }

            // Gate off voices at 4 seconds for release phase
            if (i == static_cast<int>(SAMPLE_RATE * 4.0))
            {
                sid.write(0xD404, reSIDuEngine::SAW_BITMASK);      // Voice 1 gate off
                sid.write(0xD40B, reSIDuEngine::PULSE_BITMASK);    // Voice 2 gate off
                sid.write(0xD412, reSIDuEngine::TRI_BITMASK);      // Voice 3 gate off
            }
        }

        std::cout << "\rProgress: 100%    \n\n";
        std::cout << "Sample range: " << minSample << " to " << maxSample << "\n";
        std::cout << "Audio written to: reSIDuEngine_output.wav\n";
        std::cout << "Success!\n";
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
