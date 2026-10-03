#include "tracker/kalman_filter.hpp"

namespace tracker {

namespace {

// Measurement model: we observe position only.
constexpr Matrix<2, 4> make_h() noexcept {
    Matrix<2, 4> h;
    h(0, 0) = 1.0;
    h(1, 1) = 1.0;
    return h;
}

constexpr Matrix<2, 4> kH = make_h();

}  // namespace

KalmanFilter::KalmanFilter(const KalmanConfig& config, const Measurement& first) noexcept
    : config_(config) {
    x_(0, 0) = first.x;
    x_(1, 0) = first.y;
    // Velocity unknown at start -> zero with large uncertainty.
    P_(0, 0) = config_.measurement_noise;
    P_(1, 1) = config_.measurement_noise;
    P_(2, 2) = config_.initial_velocity_variance;
    P_(3, 3) = config_.initial_velocity_variance;
}

void KalmanFilter::predict(double dt) noexcept {
    // State transition F for constant velocity.
    auto f = StateCov::identity();
    f(0, 2) = dt;
    f(1, 3) = dt;

    // Discrete white-noise acceleration model for Q.
    const double q = config_.process_noise;
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt / 2.0;
    const double dt4 = dt2 * dt2 / 4.0;
    StateCov qm;
    qm(0, 0) = dt4 * q;
    qm(1, 1) = dt4 * q;
    qm(0, 2) = dt3 * q;
    qm(2, 0) = dt3 * q;
    qm(1, 3) = dt3 * q;
    qm(3, 1) = dt3 * q;
    qm(2, 2) = dt2 * q;
    qm(3, 3) = dt2 * q;

    x_ = f * x_;
    P_ = f * P_ * f.transpose() + qm;
}

bool KalmanFilter::update(const Measurement& z) noexcept {
    Vector<2> zv;
    zv(0, 0) = z.x;
    zv(1, 0) = z.y;

    auto r = Matrix<2, 2>::identity();
    r(0, 0) = config_.measurement_noise;
    r(1, 1) = config_.measurement_noise;

    const Vector<2> innovation = zv - kH * x_;
    const Matrix<2, 2> s = kH * P_ * kH.transpose() + r;

    const auto s_inv = inverse(s);
    if (!s_inv) {
        return false;
    }

    const Matrix<4, 2> k = P_ * kH.transpose() * *s_inv;
    x_ += k * innovation;

    // Joseph form keeps P symmetric and positive semi-definite.
    const StateCov i_kh = StateCov::identity() - k * kH;
    P_ = i_kh * P_ * i_kh.transpose() + k * r * k.transpose();
    return true;
}

TrackState KalmanFilter::state() const noexcept { return {x_(0, 0), x_(1, 0), x_(2, 0), x_(3, 0)}; }

}  // namespace tracker
