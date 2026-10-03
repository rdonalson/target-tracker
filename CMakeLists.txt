cmake_minimum_required(VERSION 3.21)

project(target_tracker
    VERSION 0.1.0
    DESCRIPTION "2D constant-velocity Kalman filter target tracker"
    LANGUAGES CXX)

# ---------------------------------------------------------------------------
# Global settings
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)   # for clang-tidy / IntelliSense

option(TRACKER_BUILD_TESTS     "Build unit tests"                 ON)
option(TRACKER_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" ON)
option(TRACKER_ENABLE_ASAN     "Build with AddressSanitizer"      OFF)

# ---------------------------------------------------------------------------
# Shared compiler warnings (applied per-target, not globally)
# ---------------------------------------------------------------------------
add_library(tracker_warnings INTERFACE)
if(MSVC)
    target_compile_options(tracker_warnings INTERFACE /W4 /permissive- /utf-8)
    if(TRACKER_WARNINGS_AS_ERRORS)
        target_compile_options(tracker_warnings INTERFACE /WX)
    endif()
else()
    target_compile_options(tracker_warnings INTERFACE
        -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion
        -Wnon-virtual-dtor -Wold-style-cast -Wdouble-promotion)
    if(TRACKER_WARNINGS_AS_ERRORS)
        target_compile_options(tracker_warnings INTERFACE -Werror)
    endif()
endif()

if(TRACKER_ENABLE_ASAN)
    if(MSVC)
        target_compile_options(tracker_warnings INTERFACE /fsanitize=address)
    else()
        target_compile_options(tracker_warnings INTERFACE -fsanitize=address,undefined -fno-omit-frame-pointer)
        target_link_options(tracker_warnings INTERFACE -fsanitize=address,undefined)
    endif()
endif()

# ---------------------------------------------------------------------------
# Core library (header + source, no heap allocation in the filter)
# ---------------------------------------------------------------------------
add_library(tracker
    src/kalman_filter.cpp
    src/target_simulator.cpp)
target_include_directories(tracker PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(tracker PRIVATE tracker_warnings)

# ---------------------------------------------------------------------------
# Demo application
# ---------------------------------------------------------------------------
add_executable(tracker_demo app/main.cpp)
target_link_libraries(tracker_demo PRIVATE tracker tracker_warnings)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(TRACKER_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
