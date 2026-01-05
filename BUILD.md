# Build Instructions

## Build System

This project uses **CMake** as the build system.

## Building

```bash
# Configure (first time or after CMakeLists.txt changes)
mkdir -p build
cd build
cmake ..

# Build from project root
cmake --build build
```

## Running Examples

```bash
# Example 1: Generates reSIDuEngine_output.wav (C major chord demo)
./build/examples/reSIDuEngine_example1

# Simple test: Generates output.raw
./build/examples/simple_test

# Play raw audio (requires aplay)
aplay -f S16_LE -r 44100 -c 1 output.raw

# SID Player: Play .sid files (Commodore 64 music format)
# Requires SDL2, PortAudio, or miniaudio to be installed
./build/examples/sid_player song.sid [subtune] [chip_model] [seconds]
./build/examples/sid_player_portaudio song.sid 1 8580
./build/examples/sid_player_miniaudio song.sid
```

## Clean Build

```bash
rm -rf build
mkdir build
cd build
cmake ..
cmake --build .
```
