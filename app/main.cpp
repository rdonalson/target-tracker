#include <cmath>
#include <format>
#include <iostream>

#include "tracker/kalman_filter.hpp"
#include "tracker/target_simulator.hpp"

int main() {
    using namespace tracker;

    constexpr double kDt = 0.1;            // 10 Hz sensor
    constexpr int kSteps = 100;            // 10 seconds
    constexpr double kSensorStddev = 5.0;  // metres

    TargetSimulator sim({0.0, 0.0, 30.0, 10.0}, kSensorStddev, 42U);
    KalmanFilter kf({.process_noise = 0.5,
                     .measurement_noise = kSensorStddev * kSensorStddev,
                     .initial_velocity_variance = 1000.0},
                    sim.measure());

    std::cout << std::format("{:>6} {:>10} {:>10} {:>10} {:>10} {:>10}\n", "t(s)", "raw_err",
                             "kf_err", "est_vx", "est_vy", "true_v");

    double raw_sq = 0.0;
    double kf_sq = 0.0;
    for (int i = 1; i <= kSteps; ++i) {
        sim.step(kDt);
        kf.predict(kDt);
        const Measurement z = sim.measure();
        if (!kf.update(z)) {
            std::cerr << std::format("update skipped at step {}\n", i);
        }

        const TrackState& t = sim.truth();
        const TrackState e = kf.state();
        const double raw_err = std::hypot(z.x - t.x, z.y - t.y);
        const double kf_err = std::hypot(e.x - t.x, e.y - t.y);
        raw_sq += raw_err * raw_err;
        kf_sq += kf_err * kf_err;

        if (i % 10 == 0) {
            std::cout << std::format("{:6.1f} {:10.2f} {:10.2f} {:10.2f} {:10.2f} {:10.2f}\n",
                                     i * kDt, raw_err, kf_err, e.vx, e.vy, std::hypot(t.vx, t.vy));
        }
    }

    std::cout << std::format(
        "\nRMS position error  raw sensor: {:.2f} m   Kalman filter: {:.2f} m\n",
        std::sqrt(raw_sq / kSteps), std::sqrt(kf_sq / kSteps));
    return 0;
}
