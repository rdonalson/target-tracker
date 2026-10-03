#include "tracker/target_simulator.hpp"

namespace tracker {

TargetSimulator::TargetSimulator(TrackState initial, double sensor_stddev, std::uint32_t seed)
    : truth_(initial), rng_(seed), noise_(0.0, sensor_stddev) {}

void TargetSimulator::step(double dt) noexcept {
    truth_.x += truth_.vx * dt;
    truth_.y += truth_.vy * dt;
}

Measurement TargetSimulator::measure() {
    return {truth_.x + noise_(rng_), truth_.y + noise_(rng_)};
}

}  // namespace tracker
