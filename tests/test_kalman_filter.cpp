#include <gtest/gtest.h>

#include <cmath>

#include "tracker/kalman_filter.hpp"
#include "tracker/target_simulator.hpp"

using namespace tracker;

namespace {

constexpr double kDt = 0.1;
constexpr double kSensorStddev = 5.0;

KalmanConfig default_config() {
    return {.process_noise = 0.5,
            .measurement_noise = kSensorStddev * kSensorStddev,
            .initial_velocity_variance = 1000.0};
}

}  // namespace

TEST(KalmanFilter, InitialisesFromFirstMeasurement) {
    const KalmanFilter kf(default_config(), {10.0, -5.0});
    const auto s = kf.state();
    EXPECT_DOUBLE_EQ(s.x, 10.0);
    EXPECT_DOUBLE_EQ(s.y, -5.0);
    EXPECT_DOUBLE_EQ(s.vx, 0.0);
    EXPECT_DOUBLE_EQ(s.vy, 0.0);
}

TEST(KalmanFilter, PredictGrowsUncertainty) {
    KalmanFilter kf(default_config(), {0.0, 0.0});
    const double before = kf.covariance()(0, 0);
    kf.predict(1.0);
    EXPECT_GT(kf.covariance()(0, 0), before);
}

TEST(KalmanFilter, UpdateShrinksUncertainty) {
    KalmanFilter kf(default_config(), {0.0, 0.0});
    kf.predict(kDt);
    const double before = kf.covariance()(0, 0);
    ASSERT_TRUE(kf.update({0.0, 0.0}));
    EXPECT_LT(kf.covariance()(0, 0), before);
}

TEST(KalmanFilter, ConvergesOnTrueVelocity) {
    TargetSimulator sim({0.0, 0.0, 30.0, 10.0}, kSensorStddev, 1234U);
    KalmanFilter kf(default_config(), sim.measure());

    for (int i = 0; i < 200; ++i) {
        sim.step(kDt);
        kf.predict(kDt);
        ASSERT_TRUE(kf.update(sim.measure()));
    }

    const auto s = kf.state();
    EXPECT_NEAR(s.vx, 30.0, 2.0);
    EXPECT_NEAR(s.vy, 10.0, 2.0);
}

TEST(KalmanFilter, BeatsRawSensorAccuracy) {
    TargetSimulator sim({100.0, 50.0, -20.0, 15.0}, kSensorStddev, 99U);
    KalmanFilter kf(default_config(), sim.measure());

    double raw_sq = 0.0;
    double kf_sq = 0.0;
    constexpr int kSteps = 300;
    for (int i = 0; i < kSteps; ++i) {
        sim.step(kDt);
        kf.predict(kDt);
        const auto z = sim.measure();
        ASSERT_TRUE(kf.update(z));
        const auto& t = sim.truth();
        const auto e = kf.state();
        raw_sq += std::pow(std::hypot(z.x - t.x, z.y - t.y), 2);
        kf_sq += std::pow(std::hypot(e.x - t.x, e.y - t.y), 2);
    }
    EXPECT_LT(kf_sq, raw_sq * 0.5);  // at least ~30% lower RMS error
}

TEST(KalmanFilter, CovarianceStaysSymmetric) {
    KalmanFilter kf(default_config(), {0.0, 0.0});
    for (int i = 0; i < 50; ++i) {
        kf.predict(kDt);
        ASSERT_TRUE(kf.update({static_cast<double>(i), 0.0}));
    }
    const auto& p = kf.covariance();
    for (std::size_t r = 0; r < 4; ++r) {
        for (std::size_t c = 0; c < 4; ++c) {
            EXPECT_NEAR(p(r, c), p(c, r), 1e-9);
        }
    }
}
