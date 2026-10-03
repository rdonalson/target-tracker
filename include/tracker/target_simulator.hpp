#pragma once

#include <cstdint>
#include <random>

#include "tracker/kalman_filter.hpp"

namespace tracker {

/// Simulates a target moving at constant velocity, observed by a noisy
/// position sensor. Deterministic for a given seed so tests are repeatable.
class TargetSimulator {
public:
    TargetSimulator(TrackState initial, double sensor_stddev, std::uint32_t seed);

    /// Advance the true target by dt seconds.
    void step(double dt) noexcept;

    [[nodiscard]] const TrackState& truth() const noexcept { return truth_; }

    /// Produce a noisy measurement of the current true position.
    [[nodiscard]] Measurement measure();

private:
    TrackState truth_;
    std::mt19937 rng_;
    std::normal_distribution<double> noise_;
};

}  // namespace tracker
