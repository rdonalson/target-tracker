# Target Tracker

![CI](https://github.com/YOUR_GITHUB_USERNAME/target-tracker/actions/workflows/ci.yml/badge.svg)

A 2D target tracker in modern C++20. A simulated sensor reports noisy target
positions, and a linear Kalman filter fuses them into a smoothed estimate of
position **and velocity**.

```
  t(s)    raw_err     kf_err     est_vx     est_vy     true_v
   1.0       5.70       3.35      35.16       5.60      31.62
   5.0       4.61       1.85      29.84      10.01      31.62
  10.0       7.85       1.83      30.37       9.60      31.62

RMS position error  raw sensor: 6.89 m   Kalman filter: 2.33 m
```

The filter cuts position error by about **66%** versus the raw sensor and recovers
the target's velocity (true value 30, 10 m/s), which the sensor never measures directly.

## Design decisions

| Decision | Why |
|---|---|
| Fixed-size, stack-allocated `Matrix<R, C>` | No heap allocation after startup, which real-time and safety-critical code requires. Dimension mismatches are **compile errors**. |
| `constexpr` matrix math | Verified at compile time with `static_assert` in the tests. |
| `std::optional` for matrix inverse | A singular matrix is reported to the caller instead of dividing by zero or throwing. |
| Joseph-form covariance update | Keeps the covariance symmetric and positive semi-definite despite floating-point rounding. A unit test checks the symmetry. |
| `noexcept` throughout the filter | No exceptions on the hot path. |
| Seeded simulator | Deterministic, repeatable tests. |

### Motion model

- State: `[x, y, vx, vy]`, constant-velocity model with discrete white-noise acceleration.
- Measurement: `[x, y]`, position only.

## Building

The build requires CMake 3.21+ and a C++20 compiler (MSVC 2022, GCC 13+ or Clang 17+).
GoogleTest is picked up from vcpkg or the system if installed; otherwise it is downloaded automatically.

### Visual Studio 2022
*File → Open → Folder*, select this folder. Visual Studio reads `CMakePresets.json`.
Pick **Windows MSVC Debug**, build, and run tests from *Test Explorer*.

### VS Code
Install the recommended extensions when prompted (C/C++, CMake Tools).
Select a preset from the status bar, then *CMake: Build* and *CMake: Run Tests*.

### Command line / WSL
```bash
cmake --preset linux-gcc-debug
cmake --build --preset linux-gcc-debug
ctest --preset linux-gcc-debug
./build/linux-gcc-debug/tracker_demo
```

Presets: `windows-msvc-debug`, `windows-msvc-release`, `linux-gcc-debug`,
`linux-clang-asan` (AddressSanitizer + UndefinedBehaviorSanitizer).

## Quality checks

Every push runs these through GitHub Actions:

- Builds and tests on **Windows (MSVC)** and **Linux (GCC, and Clang with sanitizers)**
- All compiler warnings treated as errors (`/W4 /WX`, `-Wall -Wextra -Wconversion -Werror` …)
- `clang-format` style check
- `clang-tidy` static analysis (bugprone, CERT, C++ Core Guidelines, performance)

## Project layout

```
include/tracker/   public headers (matrix, Kalman filter, simulator)
src/               implementation
app/               demo executable
tests/             GoogleTest unit tests
```

## Roadmap

- [ ] Multiple targets with track-to-measurement association (nearest neighbour → GNN)
- [ ] Track initiation and deletion logic (M-of-N)
- [ ] Extended Kalman filter for range/bearing (radar-style) measurements
- [ ] Benchmark of predict/update latency with Google Benchmark
