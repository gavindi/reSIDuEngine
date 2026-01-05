// sidplayer2 - SID file player using reSIDuEngine
//
// A C64 SID music player with 6502/6510 CPU emulation and SID synthesis.
// Uses reSIDuEngine for accurate SID chip emulation.
//
// IMPORTANT: This player only supports single-SID files and will reject
// multi-SID files with an error message.
//
// Audio output using miniaudio: https://github.com/mackron/miniaudio

// reSIDuEngine - C++ implementation of the Commodore 64 SID chip

/*
 * Copyright (c) 2026, Gavin Graham <gavindi@gmail.com>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public License, v. 2.0.
 * If a copy of the MPL was not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */


#define MINIAUDIO_IMPLEMENTATION
#include "miniaudio.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <unistd.h>
#include <memory>
#include <atomic>
#include <chrono>
#include <thread>
#ifdef __unix__
#include <fcntl.h>
#endif
#include "../Source/reSIDuEngine.h"

typedef unsigned char byte;

//============================================================================
// Constants
//============================================================================

// Note: C64_PAL_CPUCLK and PAL_FRAMERATE are defined in reSIDuEngine namespace
// We'll use those directly to avoid ambiguity
const int SID_CHANNEL_AMOUNT = 3;

// Bitmasks for control registers
enum {
    GATE_BITMASK = 0x01,
    SYNC_BITMASK = 0x02,
    RING_BITMASK = 0x04,
    TEST_BITMASK = 0x08,
    TRI_BITMASK = 0x10,
    SAW_BITMASK = 0x20,
    PULSE_BITMASK = 0x40,
    NOISE_BITMASK = 0x80
};

//============================================================================
// Global variables
//============================================================================
// THREADING MODEL:
// - Main thread: Loads SID file, initializes emulation, waits for user input
// - Audio thread: Runs CPU emulation and generates audio samples (audio_callback)
//
// THREAD SAFETY:
// - loaded/initialized: Atomic flags with acquire/release semantics for synchronization
// - sidInstance: Created by main thread, accessed by audio thread (safe: init before start)
// - CPU state (PC, A, X, Y, memory, etc.): Accessed ONLY by audio thread
// - SID metadata (title, author, etc.): Read-only after loading (safe)
//
// INITIALIZATION ORDER:
// 1. Main thread loads SID file
// 2. Main thread creates sidInstance
// 3. Main thread sets loaded = true (with release barrier)
// 4. Main thread calls init() which sets initialized = true (with release barrier)
// 5. Main thread calls ma_device_start() - audio thread starts here
// 6. Audio thread checks loaded/initialized (with acquire barrier)
//============================================================================

// SID file metadata
byte SIDtitle[0x20];
byte SIDauthor[0x20];
byte SIDinfo[0x20];
byte timermode[0x20];

// Memory and addresses
byte memory[65536];
unsigned int loadaddr = 0x1000;
unsigned int initaddr = 0x1000;
unsigned int playaddf = 0x1003;
unsigned int playaddr = 0x1003;
int subtune = 0;
int subtune_amount = 1;

// SID chip configuration
int preferred_SID_model = 8580;
int SID_model = 8580;

// Timing
long samplerate = 44100;
double clk_ratio = reSIDuEngine::C64_PAL_CPUCLK / 44100.0;
double frame_sampleperiod = 44100.0 / reSIDuEngine::PAL_FRAMERATE;

// Playback state
double framecnt = 1.0;
int finished = 0;
double CPUtime = 0.0;

// Thread-safe flags (accessed by both main thread and audio callback thread)
std::atomic<bool> initialized{false};
std::atomic<bool> loaded{false};

// CPU registers and state
unsigned int PC = 0;     // Program Counter
unsigned int pPC = 0;    // Previous PC
short A = 0;             // Accumulator
short T = 0;             // Temporary register
byte X = 0;              // X register
byte Y = 0;              // Y register
byte SP = 0xFF;          // Stack Pointer
byte IR = 0;             // Instruction Register
unsigned int addr = 0;   // Address for current instruction
byte ST = 0x00;          // Status flags (N V - B D I Z C)
int cycles = 0;          // Cycle count for current instruction
unsigned int storadd = 0; // Storage address (for detecting writes)

// CPU flag constants
const byte flagsw[] = {0x01, 0x21, 0x04, 0x24, 0x00, 0x40, 0x08, 0x28};
const byte branchflag[] = {0x80, 0x40, 0x01, 0x02};

// reSIDuEngine instance
using namespace reSIDuEngine;
std::unique_ptr<SID> sidInstance;

// miniaudio device
ma_device device;
int tunelength = -1;

//============================================================================
// Function prototypes
//============================================================================

void initCPU(unsigned int mempos);
void initSID();
void init(int subt);
byte CPU();
void audio_callback(ma_device* pDevice, void* pOutput, const void* pInput, ma_uint32 frameCount);
void printSIDRegisters();

//============================================================================
// CPU Emulation
//============================================================================

void initCPU(unsigned int mempos) {
    PC = mempos;
    A = 0;
    X = 0;
    Y = 0;
    ST = 0;
    SP = 0xFF;
}

// CPU emulation function - implements 6502/6510 instruction set
byte CPU() {
    IR = memory[PC];
    cycles = 2;
    storadd = 0;

    // Instructions with bit 0 set (nybble2: 1/5/9/D:accu.instructions, 3/7/B/F:illegal opcodes)
    if (IR & 1) {
        // Addressing modes
        switch (IR & 0x1F) {
            case 1: case 3:
                ++PC;
                addr = memory[memory[PC] + X] + memory[memory[PC] + X + 1] * 256;
                cycles = 6;
                break; // (zp,x)
            case 0x11: case 0x13:
                ++PC;
                addr = memory[memory[PC]] + memory[memory[PC] + 1] * 256 + Y;
                cycles = 6;
                break; // (zp),y
            case 0x19: case 0x1F:
                ++PC;
                addr = memory[PC] + memory[PC + 1] * 256 + Y;
                ++PC;
                cycles = 5;
                break; // abs,y
            case 0x1D:
                ++PC;
                addr = memory[PC] + memory[PC + 1] * 256 + X;
                ++PC;
                cycles = 5;
                break; // abs,x
            case 0xD: case 0xF:
                ++PC;
                addr = memory[PC] + memory[PC + 1] * 256;
                ++PC;
                cycles = 4;
                break; // abs
            case 0x15:
                addr = memory[++PC] + X;
                cycles = 4;
                break; // zp,x
            case 5: case 7:
                addr = memory[++PC];
                cycles = 3;
                break; // zp
            case 0x17:
                addr = memory[++PC] + Y;
                cycles = 4;
                break; // zp,y for LAX/SAX illegal opcodes
            case 9: case 0xB:
                addr = ++PC;
                cycles = 2; // immediate
        }

        addr &= 0xFFFF;

        // Execute instruction based on upper bits
        switch (IR & 0xE0) {
            case 0x60: // ADC
                T = A;
                A += memory[addr] + (ST & 1);
                ST &= 20;
                ST |= (A & 128) | (A > 255);
                A &= 0xFF;
                ST |= (!A) << 1 | (!((T ^ memory[addr]) & 0x80) && ((T ^ A) & 0x80)) >> 1;
                break;
            case 0xE0: // SBC
                T = A;
                A -= memory[addr] + !(ST & 1);
                ST &= 20;
                ST |= (A & 128) | (A >= 0);
                A &= 0xFF;
                ST |= (!A) << 1 | (((T ^ memory[addr]) & 0x80) && ((T ^ A) & 0x80)) >> 1;
                break;
            case 0xC0: // CMP
                T = A - memory[addr];
                ST &= 124;
                ST |= (!(T & 0xFF)) << 1 | (T & 128) | (T >= 0);
                break;
            case 0x00: // ORA
                A |= memory[addr];
                ST &= 125;
                ST |= (!A) << 1 | (A & 128);
                break;
            case 0x20: // AND
                A &= memory[addr];
                ST &= 125;
                ST |= (!A) << 1 | (A & 128);
                break;
            case 0x40: // EOR
                A ^= memory[addr];
                ST &= 125;
                ST |= (!A) << 1 | (A & 128);
                break;
            case 0xA0: // LDA / LAX (illegal)
                A = memory[addr];
                ST &= 125;
                ST |= (!A) << 1 | (A & 128);
                if ((IR & 3) == 3) X = A;
                break;
            case 0x80: // STA / SAX (illegal)
                memory[addr] = A & (((IR & 3) == 3) ? X : 0xFF);
                storadd = addr;
        }
    }
    // Instructions with bit 1 set
    else if (IR & 2) {
        // Addressing modes
        switch (IR & 0x1F) {
            case 0x1E:
                ++PC;
                addr = memory[PC] + memory[PC + 1] * 256 + (((IR & 0xC0) != 0x80) ? X : Y);
                ++PC;
                cycles = 5;
                break; // abs,x / abs,y
            case 0xE:
                ++PC;
                addr = memory[PC] + memory[PC + 1] * 256;
                ++PC;
                cycles = 4;
                break; // abs
            case 0x16:
                addr = memory[++PC] + (((IR & 0xC0) != 0x80) ? X : Y);
                cycles = 4;
                break; // zp,x / zp,y
            case 6:
                addr = memory[++PC];
                cycles = 3;
                break; // zp
            case 2:
                addr = ++PC;
                cycles = 2; // imm.
        }
        addr &= 0xFFFF;

        switch (IR & 0xE0) {
            case 0x00:
                ST &= 0xFE;
            case 0x20: // ASL/ROL
                if ((IR & 0xF) == 0xA) { // Accumulator
                    A = (A << 1) + (ST & 1);
                    ST &= 60;
                    ST |= (A & 128) | (A > 255);
                    A &= 0xFF;
                    ST |= (!A) << 1;
                } else { // Memory
                    T = (memory[addr] << 1) + (ST & 1);
                    ST &= 60;
                    ST |= (T & 128) | (T > 255);
                    T &= 0xFF;
                    ST |= (!T) << 1;
                    memory[addr] = T;
                    cycles += 2;
                }
                break;
            case 0x40:
                ST &= 0xFE;
            case 0x60: // LSR/ROR
                if ((IR & 0xF) == 0xA) { // Accumulator
                    T = A;
                    A = (A >> 1) + (ST & 1) * 128;
                    ST &= 60;
                    ST |= (A & 128) | (T & 1);
                    A &= 0xFF;
                    ST |= (!A) << 1;
                } else { // Memory
                    T = (memory[addr] >> 1) + (ST & 1) * 128;
                    ST &= 60;
                    ST |= (T & 128) | (memory[addr] & 1);
                    T &= 0xFF;
                    ST |= (!T) << 1;
                    memory[addr] = T;
                    cycles += 2;
                }
                break;
            case 0xC0: // DEC/DEX
                if (IR & 4) {
                    memory[addr]--;
                    memory[addr] &= 0xFF;
                    ST &= 125;
                    ST |= (!memory[addr]) << 1 | (memory[addr] & 128);
                    cycles += 2;
                } else {
                    X--;
                    X &= 0xFF;
                    ST &= 125;
                    ST |= (!X) << 1 | (X & 128);
                }
                break;
            case 0xA0: // LDX/TSX/TAX
                if ((IR & 0xF) != 0xA)
                    X = memory[addr];
                else if (IR & 0x10) {
                    X = SP;
                    break;
                } else
                    X = A;
                ST &= 125;
                ST |= (!X) << 1 | (X & 128);
                break;
            case 0x80: // STX/TXS/TXA
                if (IR & 4) {
                    memory[addr] = X;
                    storadd = addr;
                } else if (IR & 0x10)
                    SP = X;
                else {
                    A = X;
                    ST &= 125;
                    ST |= (!A) << 1 | (A & 128);
                }
                break;
            case 0xE0: // INC/NOP
                if (IR & 4) {
                    memory[addr]++;
                    memory[addr] &= 0xFF;
                    ST &= 125;
                    ST |= (!memory[addr]) << 1 | (memory[addr] & 128);
                    cycles += 2;
                }
        }
    }
    // Register/status instructions
    else if ((IR & 0xC) == 8) {
        switch (IR & 0xF0) {
            case 0x60: // PLA
                SP++;
                SP &= 0xFF;
                A = memory[0x100 + SP];
                ST &= 125;
                ST |= (!A) << 1 | (A & 128);
                cycles = 4;
                break;
            case 0xC0: // INY
                Y++;
                Y &= 0xFF;
                ST &= 125;
                ST |= (!Y) << 1 | (Y & 128);
                break;
            case 0xE0: // INX
                X++;
                X &= 0xFF;
                ST &= 125;
                ST |= (!X) << 1 | (X & 128);
                break;
            case 0x80: // DEY
                Y--;
                Y &= 0xFF;
                ST &= 125;
                ST |= (!Y) << 1 | (Y & 128);
                break;
            case 0x00: // PHP
                memory[0x100 + SP] = ST;
                SP--;
                SP &= 0xFF;
                cycles = 3;
                break;
            case 0x20: // PLP
                SP++;
                SP &= 0xFF;
                ST = memory[0x100 + SP];
                cycles = 4;
                break;
            case 0x40: // PHA
                memory[0x100 + SP] = A;
                SP--;
                SP &= 0xFF;
                cycles = 3;
                break;
            case 0x90: // TYA
                A = Y;
                ST &= 125;
                ST |= (!A) << 1 | (A & 128);
                break;
            case 0xA0: // TAY
                Y = A;
                ST &= 125;
                ST |= (!Y) << 1 | (Y & 128);
                break;
            default: // CLC/SEC/CLI/SEI/CLV/CLD/SED
                if (flagsw[IR >> 5] & 0x20)
                    ST |= (flagsw[IR >> 5] & 0xDF);
                else
                    ST &= 255 - (flagsw[IR >> 5] & 0xDF);
        }
    }
    // Control/branch/compare instructions
    else {
        // Branch instructions
        if ((IR & 0x1F) == 0x10) {
            PC++;
            T = memory[PC];
            if (T & 0x80) T -= 0x100;
            if (IR & 0x20) {
                if (ST & branchflag[IR >> 6]) {
                    PC += T;
                    cycles = 3;
                }
            } else {
                if (!(ST & branchflag[IR >> 6])) {
                    PC += T;
                    cycles = 3;
                }
            }
        } else {
            // Addressing modes
            switch (IR & 0x1F) {
                case 0:
                    addr = ++PC;
                    cycles = 2;
                    break; // imm. (or abs.low for JSR/BRK)
                case 0x1C:
                    ++PC;
                    addr = memory[PC] + memory[PC + 1] * 256 + X;
                    ++PC;
                    cycles = 5;
                    break; // abs,x
                case 0xC:
                    ++PC;
                    addr = memory[PC] + memory[PC + 1] * 256;
                    ++PC;
                    cycles = 4;
                    break; // abs
                case 0x14:
                    addr = memory[++PC] + X;
                    cycles = 4;
                    break; // zp,x
                case 4:
                    addr = memory[++PC];
                    cycles = 3; // zp
            }
            addr &= 0xFFFF;

            switch (IR & 0xE0) {
                case 0x00: // BRK
                    memory[0x100 + SP] = PC % 256;
                    SP--;
                    SP &= 0xFF;
                    memory[0x100 + SP] = PC / 256;
                    SP--;
                    SP &= 0xFF;
                    memory[0x100 + SP] = ST;
                    SP--;
                    SP &= 0xFF;
                    PC = memory[0xFFFE] + memory[0xFFFF] * 256 - 1;
                    cycles = 7;
                    break;
                case 0x20: // BIT/JSR
                    if (IR & 0xF) {
                        ST &= 0x3D;
                        ST |= (memory[addr] & 0xC0) | (!(A & memory[addr])) << 1;
                    } else { // JSR
                        memory[0x100 + SP] = (PC + 2) % 256;
                        SP--;
                        SP &= 0xFF;
                        memory[0x100 + SP] = (PC + 2) / 256;
                        SP--;
                        SP &= 0xFF;
                        PC = memory[addr] + memory[addr + 1] * 256 - 1;
                        cycles = 6;
                    }
                    break;
                case 0x40: // JMP/RTI
                    if (IR & 0xF) {
                        PC = addr - 1;
                        cycles = 3;
                    } else { // RTI
                        if (SP >= 0xFF) return 0xFE;
                        SP++;
                        SP &= 0xFF;
                        ST = memory[0x100 + SP];
                        SP++;
                        SP &= 0xFF;
                        T = memory[0x100 + SP];
                        SP++;
                        SP &= 0xFF;
                        PC = memory[0x100 + SP] + T * 256 - 1;
                        cycles = 6;
                    }
                    break;
                case 0x60: // JMP()/RTS
                    if (IR & 0xF) {
                        PC = memory[addr] + memory[addr + 1] * 256 - 1;
                        cycles = 5;
                    } else { // RTS
                        if (SP >= 0xFF) return 0xFF;
                        SP++;
                        SP &= 0xFF;
                        T = memory[0x100 + SP];
                        SP++;
                        SP &= 0xFF;
                        PC = memory[0x100 + SP] + T * 256 - 1;
                        cycles = 6;
                    }
                    break;
                case 0xC0: // CPY
                    T = Y - memory[addr];
                    ST &= 124;
                    ST |= (!(T & 0xFF)) << 1 | (T & 128) | (T >= 0);
                    break;
                case 0xE0: // CPX
                    T = X - memory[addr];
                    ST &= 124;
                    ST |= (!(T & 0xFF)) << 1 | (T & 128) | (T >= 0);
                    break;
                case 0xA0: // LDY
                    Y = memory[addr];
                    ST &= 125;
                    ST |= (!Y) << 1 | (Y & 128);
                    break;
                case 0x80: // STY
                    memory[addr] = Y;
                    storadd = addr;
            }
        }
    }

    PC++;
    PC &= 0xFFFF;
    return 0;
}

//============================================================================
// SID Initialization
//============================================================================

void initSID() {
    // Clear SID and I/O memory regions
    for (int i = 0xD400; i <= 0xD7FF; i++)
        memory[i] = 0;
    for (int i = 0xDE00; i <= 0xDFFF; i++)
        memory[i] = 0;

    // Reset reSIDuEngine instance
    if (sidInstance)
        sidInstance->reset();
}

//============================================================================
// Initialization
//============================================================================

void init(int subt) {
    if (!loaded.load(std::memory_order_acquire))
        return;

    initialized.store(false, std::memory_order_relaxed);
    subtune = subt;
    initCPU(initaddr);
    initSID();

    A = subtune;
    memory[1] = 0x37;  // Memory banking configuration
    memory[0xDC05] = 0;

    // Execute init routine with timeout
    for (int timeout = 100000; timeout >= 0; timeout--) {
        if (CPU())
            break;
    }

    // Determine frame timing: CIA timer or VSync
    if (timermode[subtune] || memory[0xDC05]) {
        if (!memory[0xDC05]) {
            memory[0xDC04] = 0x24;
            memory[0xDC05] = 0x40;
        }
        frame_sampleperiod = (memory[0xDC04] + memory[0xDC05] * 256) / clk_ratio;
    } else {
        frame_sampleperiod = samplerate / reSIDuEngine::PAL_FRAMERATE;
    }

    // Determine play address
    if (playaddf == 0) {
        playaddr = ((memory[1] & 3) < 2) ? memory[0xFFFE] + memory[0xFFFF] * 256
                                         : memory[0x314] + memory[0x315] * 256;
    } else {
        playaddr = playaddf;
        if (playaddr >= 0xE000 && memory[1] == 0x37)
            memory[1] = 0x35; // Crystal Kingdom Dizzy workaround
    }

    initCPU(playaddr);

    framecnt = 1;
    finished = 0;
    CPUtime = 0;

    // Sync SID registers to reSIDuEngine after init
    if (sidInstance) {
        for (int reg = 0; reg < 0x19; reg++) {
            sidInstance->write(0xD400 + reg, memory[0xD400 + reg]);
        }
    }

    // Set initialized flag with release semantics to ensure all initialization
    // is visible to the audio callback thread
    initialized.store(true, std::memory_order_release);
}

//============================================================================
// Audio callback
//============================================================================
// THREAD SAFETY: This callback runs on a separate audio thread managed by miniaudio.
// - All CPU emulation state (PC, A, X, Y, memory, etc.) is accessed only by this thread
// - sidInstance is initialized by main thread before ma_device_start() is called
// - loaded/initialized flags use atomic operations with acquire/release semantics

void audio_callback(ma_device* pDevice, void* pOutput, const void* pInput, ma_uint32 frameCount) {
    int16_t* stream = (int16_t*)pOutput;

    (void)pInput;
    (void)pDevice;

    // Use acquire semantics to ensure we see all initialization from main thread
    if (!loaded.load(std::memory_order_acquire) || !initialized.load(std::memory_order_acquire)) {
        // Output silence if not ready
        memset(stream, 0, frameCount * sizeof(int16_t));
        return;
    }

    for (ma_uint32 i = 0; i < frameCount; i++) {
        framecnt--;

        if (framecnt <= 0) {
            framecnt = frame_sampleperiod;
            finished = 0;
            PC = playaddr;
            SP = 0xFF;
        }

        if (finished == 0) {
            while (CPUtime <= clk_ratio) {
                pPC = PC;
                if (CPU() >= 0xFE) {
                    finished = 1;
                    break;
                } else {
                    CPUtime += cycles;
                }

                // IRQ player ROM return handling
                if ((memory[1] & 3) > 1 && pPC < 0xE000 && (PC == 0xEA31 || PC == 0xEA81)) {
                    finished = 1;
                    break;
                }

                // Dynamic CIA timer adjustment (Galway/Rubicon workaround)
                if ((addr == 0xDC05 || addr == 0xDC04) && (memory[1] & 3) && timermode[subtune]) {
                    frame_sampleperiod = (memory[0xDC04] + memory[0xDC05] * 256) / clk_ratio;
                }

                // Handle writes above $D420 (CJ in the USA workaround)
                if (storadd >= 0xD420 && storadd < 0xD800 && (memory[1] & 3)) {
                    memory[storadd & 0xD41F] = memory[storadd];
                }

                // Forward SID register writes to reSIDuEngine
                if (storadd >= 0xD400 && storadd < 0xD420 && sidInstance) {
                    sidInstance->write(storadd, memory[storadd]);
                }
            }
            CPUtime -= clk_ratio;
        }

        // Generate audio sample from reSIDuEngine using reSIDfp-compatible API
        // Pass enough cycles to ensure we get exactly 1 sample
        // Adding 1.0 ensures cycles/clkRatio >= 1.0 after integer conversion
        short sample = 0;
        if (sidInstance) {
            unsigned int cycles_per_sample = static_cast<unsigned int>(clk_ratio + 1.0);
            sidInstance->clock(cycles_per_sample, &sample);
        }

        // Scale to reduce volume and prevent clipping
        int output = static_cast<int>(sample * 0.25f);

        // Clamp to prevent distortion
        if (output > 32767)
            output = 32767;
        else if (output < -32768)
            output = -32768;

        stream[i] = static_cast<int16_t>(output);
    }
}

//============================================================================
// SID Register Display
//============================================================================

void printSIDRegisters() {
    printf("\rD400: ");

    // Voice 1 (0xD400-0xD406)
    for (int i = 0; i < 7; i++) {
        printf("%02X ", memory[0xD400 + i]);
    }
    printf("| ");

    // Voice 2 (0xD407-0xD40D)
    for (int i = 0; i < 7; i++) {
        printf("%02X ", memory[0xD407 + i]);
    }
    printf("| ");

    // Voice 3 (0xD40E-0xD414)
    for (int i = 0; i < 7; i++) {
        printf("%02X ", memory[0xD40E + i]);
    }
    printf("| ");

    // Global registers (0xD415-0xD418)
    for (int i = 0; i < 4; i++) {
        printf("%02X ", memory[0xD415 + i]);
    }

    fflush(stdout);
}

//============================================================================
// Main Program
//============================================================================

int main(int argc, char* argv[]) {
    FILE* InputFile;
    byte filedata[65536];
    int readata, strend, datalen;
    unsigned int i, offs;

    printf("\nsidplayer2 - SID player using reSIDuEngine (single-SID only)\n\n");

    // Parse command line
    if (argc < 2) {
        printf("Usage: sidplayer2 <inputfile.sid> [subtune] [model] [seconds]\n");
        printf("  subtune: Subtune number (1-based, default: 1)\n");
        printf("  model:   SID chip model (6581 or 8580, default: auto)\n");
        printf("  seconds: Playback duration (default: manual stop)\n\n");
        return 1;
    }

    if (argc >= 3) {
        sscanf(argv[2], "%d", &subtune);
        subtune--; // Convert to 0-based
        if (subtune < 0 || subtune > 63)
            subtune = 0;
    } else {
        subtune = 0;
    }

    int requested_SID_model = -1;
    if (argc >= 4)
        sscanf(argv[3], "%d", &requested_SID_model);

    if (argc >= 5)
        sscanf(argv[4], "%d", &tunelength);

    // Load SID file
    InputFile = fopen(argv[1], "rb");
    if (InputFile == NULL) {
        printf("Error: File not found: %s\n", argv[1]);
        return 1;
    }

    datalen = 0;
    do {
        readata = fgetc(InputFile);
        if (readata != EOF)
            filedata[datalen++] = readata;
    } while (readata != EOF && datalen < 65536);
    fclose(InputFile);
    datalen--;

    printf("Loaded: %s (%d bytes)\n", argv[1], datalen);

    // Parse SID file header
    offs = filedata[7];
    loadaddr = filedata[8] + filedata[9] ? filedata[8] * 256 + filedata[9]
                                         : filedata[offs] + filedata[offs + 1] * 256;

    // Extract timer modes
    for (i = 0; i < 32; i++) {
        timermode[31 - i] = (filedata[0x12 + (i >> 3)] & (byte)pow(2, 7 - i % 8)) ? 1 : 0;
    }

    // Clear memory and load program data
    for (i = 0; i < 65536; i++)
        memory[i] = 0;

    for (i = offs + 2; i < datalen; i++) {
        if (loadaddr + i - (offs + 2) < 65536)
            memory[loadaddr + i - (offs + 2)] = filedata[i];
    }

    // Extract metadata strings
    strend = 1;
    for (i = 0; i < 32; i++) {
        if (strend != 0)
            strend = SIDtitle[i] = filedata[0x16 + i];
        else
            strend = SIDtitle[i] = 0;
    }

    strend = 1;
    for (i = 0; i < 32; i++) {
        if (strend != 0)
            strend = SIDauthor[i] = filedata[0x36 + i];
        else
            strend = SIDauthor[i] = 0;
    }

    strend = 1;
    for (i = 0; i < 32; i++) {
        if (strend != 0)
            strend = SIDinfo[i] = filedata[0x56 + i];
        else
            strend = SIDinfo[i] = 0;
    }

    // Extract addresses
    initaddr = filedata[0xA] + filedata[0xB] ? filedata[0xA] * 256 + filedata[0xB] : loadaddr;
    playaddr = playaddf = filedata[0xC] * 256 + filedata[0xD];
    subtune_amount = filedata[0xF];

    // Determine SID chip model
    preferred_SID_model = (filedata[0x77] & 0x30) >= 0x20 ? 8580 : 6581;

    // Check for multi-SID configuration
    unsigned int SID_address_2 = filedata[0x7A] >= 0x42 && (filedata[0x7A] < 0x80 || filedata[0x7A] >= 0xE0)
                                     ? 0xD000 + filedata[0x7A] * 16
                                     : 0;
    unsigned int SID_address_3 = filedata[0x7B] >= 0x42 && (filedata[0x7B] < 0x80 || filedata[0x7B] >= 0xE0)
                                     ? 0xD000 + filedata[0x7B] * 16
                                     : 0;

    int SIDamount = 1 + (SID_address_2 > 0) + (SID_address_3 > 0);

    // Display file info
    printf("Title:    %s\n", SIDtitle);
    printf("Author:   %s\n", SIDauthor);
    printf("Info:     %s\n", SIDinfo);
    printf("Load:     $%04X\n", loadaddr);
    printf("Init:     $%04X\n", initaddr);
    printf("Play:     $%04X\n", playaddr);
    printf("Subtunes: %d (playing #%d)\n", subtune_amount, subtune + 1);
    printf("Model:    %d (preferred)\n", preferred_SID_model);

    // REJECT MULTI-SID FILES
    if (SIDamount > 1) {
        printf("\n*** ERROR: This is a multi-SID file (%d SID chips) ***\n", SIDamount);
        printf("sidplayer2 only supports single-SID files.\n");
        printf("Please use the original sidplayer for multi-SID files.\n\n");
        if (SID_address_2)
            printf("  SID2 at: $%04X\n", SID_address_2);
        if (SID_address_3)
            printf("  SID3 at: $%04X\n", SID_address_3);
        printf("\n");
        return 2;
    }

    // Determine final SID model
    if (requested_SID_model == 8580 || requested_SID_model == 6581)
        SID_model = requested_SID_model;
    else
        SID_model = preferred_SID_model;

    printf("Using model: %d\n", SID_model);

    // Initialize reSIDuEngine
    SIDModel model = (SID_model == 8580) ? MOS8580 : MOS6581;
    sidInstance = std::make_unique<SID>(samplerate, model);
    sidInstance->setClock(reSIDuEngine::C64_PAL_CPUCLK);
    sidInstance->reset();

    // Calculate clock ratio
    clk_ratio = reSIDuEngine::C64_PAL_CPUCLK / samplerate;

    // Set loaded flag with release semantics to ensure sidInstance initialization
    // is visible to the audio callback thread
    loaded.store(true, std::memory_order_release);

    // Initialize the SID tune
    init(subtune);

    printf("Frame period: %.2f samples (%.2fHz)\n", frame_sampleperiod,
           samplerate / frame_sampleperiod);

    // Initialize miniaudio
    ma_device_config deviceConfig = ma_device_config_init(ma_device_type_playback);
    deviceConfig.playback.format = ma_format_s16;
    deviceConfig.playback.channels = 1;
    deviceConfig.sampleRate = samplerate;
    deviceConfig.dataCallback = audio_callback;
    deviceConfig.pUserData = NULL;

    if (ma_device_init(NULL, &deviceConfig, &device) != MA_SUCCESS) {
        fprintf(stderr, "Failed to initialize miniaudio device.\n");
        return 3;
    }

    if (ma_device_start(&device) != MA_SUCCESS) {
        fprintf(stderr, "Failed to start miniaudio device.\n");
        ma_device_uninit(&device);
        return 4;
    }

    printf("\nPlayback started... ");
    if (tunelength != -1) {
        printf("(playing for %d seconds)\n\n", tunelength);

        auto startTime = std::chrono::steady_clock::now();
        auto endTime = startTime + std::chrono::seconds(tunelength);

        while (std::chrono::steady_clock::now() < endTime) {
            printSIDRegisters();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        printf("\n\n");
    } else {
        printf("Press Enter to stop.\n\n");

        // Set stdin to non-blocking mode on Unix systems
        #ifdef __unix__
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
        #endif

        bool running = true;
        while (running) {
            printSIDRegisters();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            // Check for Enter key (non-blocking)
            #ifdef __unix__
            char c;
            if (read(STDIN_FILENO, &c, 1) > 0 && c == '\n') {
                running = false;
            }
            #else
            // For non-Unix systems, use blocking getchar on first iteration
            // This is a limitation, but keeps the code simple
            if (getchar() == '\n') {
                running = false;
            }
            #endif
        }

        // Restore blocking mode on Unix systems
        #ifdef __unix__
        fcntl(STDIN_FILENO, F_SETFL, flags);
        #endif

        printf("\n\n");
    }

    printf("Stopping playback...\n");
    ma_device_uninit(&device);

    return 0;
}
