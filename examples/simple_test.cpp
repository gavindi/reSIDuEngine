// * reSIDuEngine - C++ implementation of the Commodore 64 SID chip

/*
 * Copyright (c) 2026, Gavin Graham <gavindi@gmail.com>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public License, v. 2.0.
 * If a copy of the MPL was not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

/*
 * Simple test to diagnose graininess issue
 * Tests a single sawtooth wave without filter
 */

#include "reSIDuEngine.h"
#include <iostream>
#include <fstream>
#include <cstdint>

class SimpleWavWriter
{
public:
    SimpleWavWriter(const std::string& filename, int sampleRate)
        : sampleRate(sampleRate), numSamples(0)
    {
        file.open(filename, std::ios::binary);
        writeHeader(0);
    }

    ~SimpleWavWriter() { close(); }

    void writeSample(float sample)
    {
        int16_t pcm = static_cast<int16_t>(sample * 32767.0f);
        file.write(reinterpret_cast<char*>(&pcm), sizeof(pcm));
        numSamples++;
    }

    void close()
    {
        if (file.is_open())
        {
            file.seekp(0);
            writeHeader(numSamples);
            file.close();
        }
    }

private:
    void writeHeader(uint32_t samples)
    {
        uint32_t dataSize = samples * sizeof(int16_t);
        uint32_t fileSize = dataSize + 36;

        file.write("RIFF", 4);
        file.write(reinterpret_cast<char*>(&fileSize), 4);
        file.write("WAVE", 4);
        file.write("fmt ", 4);

        uint32_t fmtSize = 16;
        file.write(reinterpret_cast<char*>(&fmtSize), 4);

        uint16_t audioFormat = 1;
        file.write(reinterpret_cast<char*>(&audioFormat), 2);

        uint16_t numChannels = 1;
        file.write(reinterpret_cast<char*>(&numChannels), 2);

        uint32_t rate = sampleRate;
        file.write(reinterpret_cast<char*>(&rate), 4);

        uint32_t byteRate = sampleRate * sizeof(int16_t);
        file.write(reinterpret_cast<char*>(&byteRate), 4);

        uint16_t blockAlign = sizeof(int16_t);
        file.write(reinterpret_cast<char*>(&blockAlign), 2);

        uint16_t bitsPerSample = 16;
        file.write(reinterpret_cast<char*>(&bitsPerSample), 2);

        file.write("data", 4);
        file.write(reinterpret_cast<char*>(&dataSize), 4);
    }

    std::ofstream file;
    int sampleRate;
    uint32_t numSamples;
};

int main()
{
    std::cout << "Simple reSIDuEngine Test - Voice 2 Pulse Wave, No Filter\n";
    std::cout << "==========================================================\n\n";

    const int SAMPLE_RATE = 44100;
    const double DURATION = 2.0;
    const int NUM_SAMPLES = static_cast<int>(SAMPLE_RATE * DURATION);

    reSIDuEngine::SID sid(SAMPLE_RATE, reSIDuEngine::MOS8580);
    sid.setVolume(0.5f);  // Match the full example

    // Calculate frequency for E4 (329.63 Hz) - matching full example
    const double CLOCK_FREQ = 985248.0;
    uint16_t freq = static_cast<uint16_t>((329.63 * 16777216.0) / CLOCK_FREQ);

    std::cout << "Frequency register for E4 (329.63 Hz): " << freq << "\n";
    std::cout << "Sample rate: " << SAMPLE_RATE << " Hz\n";
    std::cout << "Duration: " << DURATION << " seconds\n\n";

    // Setup voice 2 (channel 1) with PULSE waveform - matching the full example
    sid.write(0xD407, freq & 0xFF);
    sid.write(0xD408, (freq >> 8) & 0xFF);
    sid.write(0xD409, 0x00);  // PW low
    sid.write(0xD40A, 0x08);  // PW high (0x800 = 50% duty)
    sid.write(0xD40C, 0x00);  // Fast attack/decay
    sid.write(0xD40D, 0xF0);  // Full sustain, fast release
    sid.write(0xD40B, reSIDuEngine::PULSE_BITMASK | reSIDuEngine::GATE_BITMASK);

    // NO FILTER - bypass filter completely
    sid.write(0xD417, 0x00);  // No voices to filter
    sid.write(0xD418, 0x0F);  // No filter mode, volume 15

    std::cout << "Generating audio...\n";

    SimpleWavWriter wav("simple_test.wav", SAMPLE_RATE);
    float minSample = 0.0f, maxSample = 0.0f;

    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        float sample = sid.clockSample();

        if (sample < minSample) minSample = sample;
        if (sample > maxSample) maxSample = sample;

        // Debug: print first few samples
        if (i < 10)
        {
            std::cout << "Sample " << i << ": " << sample << "\n";
        }

        wav.writeSample(sample);

        if (i % (SAMPLE_RATE / 10) == 0 && i > 0)
        {
            float progress = 100.0f * i / NUM_SAMPLES;
            std::cout << "\rProgress: " << static_cast<int>(progress) << "%" << std::flush;
        }
    }

    std::cout << "\rProgress: 100%\n\n";
    std::cout << "Sample range: " << minSample << " to " << maxSample << "\n";
    std::cout << "Audio written to: simple_test.wav\n";
    std::cout << "Done!\n";

    return 0;
}
