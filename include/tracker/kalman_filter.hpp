#pragma once

#include "tracker/matrix.hpp"

namespace tracker {

/// A 2D position measurement from a sensor (e.g. a radar return).
struct Measurement {
    double x{};
    double y{};
};

/// Estimated target state: position and velocity.
struct TrackState {
    double x{};
    double y{};
    double vx{};
    double vy{};
};

/// Tuning parameters for the filter.
struct KalmanConfig {
    double process_noise{1.0};       ///< Acceleration noise spectral density (m^2/s^3)
    double measurement_noise{25.0};  ///< Sensor position variance (m^2)
    double initial_velocity_variance{100.0};
};

/// Linear Kalman filter with a constant-velocity motion model.
///
/// State vector: [x, y, vx, vy]^T
/// Measurement:  [x, y]^T
///
/// All storage is fixed-size; no dynamic allocation after construction.
class KalmanFilter {
public:
    using StateVec = Vector<4>;
    using StateCov = Matrix<4, 4>;

    explicit KalmanFilter(const KalmanConfig& config, const Measurement& first) noexcept;

    /// Propagate the state forward by dt seconds.
    void predict(double dt) noexcept;

    /// Fuse a new measurement. Returns false if the innovation covariance
    /// was singular and the update was skipped.
    [[nodiscard]] bool update(const Measurement& z) noexcept;

    [[nodiscard]] TrackState state() const noexcept;
    [[nodiscard]] const StateCov& covariance() const noexcept { return P_; }

private:
    KalmanConfig config_;
    StateVec x_;  // Matrix zero-initialises itself
    StateCov P_;
};

}  // namespace tracker
