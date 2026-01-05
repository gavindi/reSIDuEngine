cmake_minimum_required(VERSION 3.10)

# reSIDuEngine example
add_executable(reSIDuEngine_example1 reSIDuEngine_example1.cpp)
target_link_libraries(reSIDuEngine_example1 PRIVATE residuengine)

# Simple test
add_executable(simple_test simple_test.cpp)
target_link_libraries(simple_test PRIVATE residuengine)

# SID Player with miniaudio (single-header, cross-platform)
# Note: Requires miniaudio.h in examples/ directory or include path
# Download from: https://github.com/mackron/miniaudio
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/miniaudio.h")
    # SID Player 2 (single-SID only)
    add_executable(sidplayer2 sidplayer2.cpp)
    target_link_libraries(sidplayer2 PRIVATE residuengine)
    if(UNIX)
        target_link_libraries(sidplayer2 PRIVATE pthread m dl)
    endif()
    message(STATUS "Building sidplayer2 example with miniaudio (single-SID only)")
else()
    message(WARNING "miniaudio.h not found in examples/ - sidplayer will not be built. Download from https://github.com/mackron/miniaudio")
endif()
