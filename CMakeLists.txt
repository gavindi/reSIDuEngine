cmake_minimum_required(VERSION 3.10)
project(reSIDuEngine VERSION 1.0.0 LANGUAGES CXX)

# Set default build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build (Debug or Release)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# reSIDuEngine sources
set(RESIDUENGINE_SOURCES
    Source/reSIDuEngine.cpp
)

set(RESIDUENGINE_HEADERS
    Source/reSIDuEngine.h
)

# Create reSIDuEngine static library
add_library(residuengine STATIC ${RESIDUENGINE_SOURCES} ${RESIDUENGINE_HEADERS})

target_include_directories(residuengine
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Source>
        $<INSTALL_INTERFACE:include>
)

# Compiler options for reSIDuEngine
if(MSVC)
    target_compile_options(residuengine PRIVATE /W4)
else()
    target_compile_options(residuengine PRIVATE -Wall -Wextra -pedantic)
endif()

# Optional: Build examples
option(BUILD_EXAMPLES "Build example programs" ON)

if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# Installation
install(TARGETS residuengine
    EXPORT residuengineTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(FILES ${RESIDUENGINE_HEADERS}
    DESTINATION include/residuengine
)

install(EXPORT residuengineTargets
    FILE residuengineTargets.cmake
    NAMESPACE residuengine::
    DESTINATION lib/cmake/residuengine
)

# Create package config
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/residuengineConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/residuengineConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/residuengineConfig.cmake"
    INSTALL_DESTINATION lib/cmake/residuengine
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/residuengineConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/residuengineConfigVersion.cmake"
    DESTINATION lib/cmake/residuengine
)
