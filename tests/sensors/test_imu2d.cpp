// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include <cmath>
#include <gtest/gtest.h>
#include <sensors/imu2d.hpp>

using namespace sensors;

class IMU2DTest : public ::testing::Test {
  protected:
    void SetUp() override {
        imu.set_acc_bias(Eigen::Vector2f::Zero());
        imu.set_gyr_bias(0.0f);
        imu.set_acc_noise_std(Eigen::Vector2f(0.01f, 0.01f));
        imu.set_gyr_noise_std(0.001f);
    }

    IMU2D imu;
};

// ============================================================================
// Bias configuration tests
// ============================================================================

TEST_F(IMU2DTest, DefaultBiasIsZero) {
    IMU2D default_imu;
    EXPECT_NEAR(default_imu.acc_bias().x(), 0.0f, 1e-6f);
    EXPECT_NEAR(default_imu.acc_bias().y(), 0.0f, 1e-6f);
    EXPECT_NEAR(default_imu.gyr_bias(), 0.0f, 1e-6f);
}

TEST_F(IMU2DTest, SetAccBias) {
    Eigen::Vector2f bias(0.1f, -0.2f);
    imu.set_acc_bias(bias);

    EXPECT_NEAR(imu.acc_bias().x(), 0.1f, 1e-6f);
    EXPECT_NEAR(imu.acc_bias().y(), -0.2f, 1e-6f);
}

TEST_F(IMU2DTest, SetGyrBias) {
    imu.set_gyr_bias(0.05f);
    EXPECT_NEAR(imu.gyr_bias(), 0.05f, 1e-6f);
}

// ============================================================================
// Noise configuration tests
// ============================================================================

TEST_F(IMU2DTest, SetAccNoiseStd) {
    Eigen::Vector2f noise_std(0.02f, 0.03f);
    imu.set_acc_noise_std(noise_std);

    EXPECT_NEAR(imu.acc_noise_std().x(), 0.02f, 1e-6f);
    EXPECT_NEAR(imu.acc_noise_std().y(), 0.03f, 1e-6f);
}

TEST_F(IMU2DTest, SetGyrNoiseStd) {
    imu.set_gyr_noise_std(0.002f);
    EXPECT_NEAR(imu.gyr_noise_std(), 0.002f, 1e-6f);
}

// ============================================================================
// Measurement generation tests
// ============================================================================

TEST_F(IMU2DTest, MeasureWithZeroBiasAndNoise) {
    imu.set_acc_bias(Eigen::Vector2f::Zero());
    imu.set_gyr_bias(0.0f);
    imu.set_acc_noise_std(Eigen::Vector2f::Zero());
    imu.set_gyr_noise_std(0.0f);

    Eigen::Vector2f gt_acc(1.0f, 2.0f);
    float gt_gyr = 0.5f;

    IMUMeasurement meas = imu.measure(gt_acc, gt_gyr);

    EXPECT_NEAR(meas.acc.x(), 1.0f, 1e-6f);
    EXPECT_NEAR(meas.acc.y(), 2.0f, 1e-6f);
    EXPECT_NEAR(meas.gyr, 0.5f, 1e-6f);
}

TEST_F(IMU2DTest, MeasureWithBias) {
    Eigen::Vector2f acc_bias(0.1f, 0.2f);
    float gyr_bias = 0.05f;
    imu.set_acc_bias(acc_bias);
    imu.set_gyr_bias(gyr_bias);
    imu.set_acc_noise_std(Eigen::Vector2f::Zero());
    imu.set_gyr_noise_std(0.0f);

    Eigen::Vector2f gt_acc(1.0f, 2.0f);
    float gt_gyr = 0.5f;

    IMUMeasurement meas = imu.measure(gt_acc, gt_gyr);

    EXPECT_NEAR(meas.acc.x(), 1.1f, 1e-6f);
    EXPECT_NEAR(meas.acc.y(), 2.2f, 1e-6f);
    EXPECT_NEAR(meas.gyr, 0.55f, 1e-6f);
}

TEST_F(IMU2DTest, MeasureWithNoise) {
    imu.set_acc_noise_std(Eigen::Vector2f(0.1f, 0.1f));
    imu.set_gyr_noise_std(0.01f);

    Eigen::Vector2f gt_acc(0.0f, 0.0f);
    float gt_gyr = 0.0f;

    // Collect many measurements to estimate noise statistics
    std::vector<float> acc_x, acc_y, gyr;
    const int n_samples = 1000;
    for (int i = 0; i < n_samples; i++) {
        IMUMeasurement meas = imu.measure(gt_acc, gt_gyr);
        acc_x.push_back(meas.acc.x());
        acc_y.push_back(meas.acc.y());
        gyr.push_back(meas.gyr);
    }

    // Calculate means
    float mean_acc_x = 0, mean_acc_y = 0, mean_gyr = 0;
    for (int i = 0; i < n_samples; i++) {
        mean_acc_x += acc_x[i];
        mean_acc_y += acc_y[i];
        mean_gyr += gyr[i];
    }
    mean_acc_x /= n_samples;
    mean_acc_y /= n_samples;
    mean_gyr /= n_samples;

    // Calculate std deviations
    float var_acc_x = 0, var_acc_y = 0, var_gyr = 0;
    for (int i = 0; i < n_samples; i++) {
        var_acc_x += (acc_x[i] - mean_acc_x) * (acc_x[i] - mean_acc_x);
        var_acc_y += (acc_y[i] - mean_acc_y) * (acc_y[i] - mean_acc_y);
        var_gyr += (gyr[i] - mean_gyr) * (gyr[i] - mean_gyr);
    }
    float std_acc_x = std::sqrt(var_acc_x / n_samples);
    float std_acc_y = std::sqrt(var_acc_y / n_samples);
    float std_gyr = std::sqrt(var_gyr / n_samples);

    // Means should be close to 0
    EXPECT_NEAR(mean_acc_x, 0.0f, 0.02f);
    EXPECT_NEAR(mean_acc_y, 0.0f, 0.02f);
    EXPECT_NEAR(mean_gyr, 0.0f, 0.002f);

    // Std deviations should be close to configured values
    EXPECT_NEAR(std_acc_x, 0.1f, 0.02f);
    EXPECT_NEAR(std_acc_y, 0.1f, 0.02f);
    EXPECT_NEAR(std_gyr, 0.01f, 0.002f);
}

TEST_F(IMU2DTest, MeasureWithDifferentXYNoise) {
    imu.set_acc_noise_std(Eigen::Vector2f(0.05f, 0.2f));
    imu.set_gyr_noise_std(0.0f);
    imu.set_acc_bias(Eigen::Vector2f::Zero());

    Eigen::Vector2f gt_acc(0.0f, 0.0f);
    float gt_gyr = 0.0f;

    std::vector<float> acc_x, acc_y;
    const int n_samples = 1000;
    for (int i = 0; i < n_samples; i++) {
        IMUMeasurement meas = imu.measure(gt_acc, gt_gyr);
        acc_x.push_back(meas.acc.x());
        acc_y.push_back(meas.acc.y());
    }

    // Calculate variances
    float mean_x = 0, mean_y = 0;
    for (int i = 0; i < n_samples; i++) {
        mean_x += acc_x[i];
        mean_y += acc_y[i];
    }
    mean_x /= n_samples;
    mean_y /= n_samples;

    float var_x = 0, var_y = 0;
    for (int i = 0; i < n_samples; i++) {
        var_x += (acc_x[i] - mean_x) * (acc_x[i] - mean_x);
        var_y += (acc_y[i] - mean_y) * (acc_y[i] - mean_y);
    }
    float std_x = std::sqrt(var_x / n_samples);
    float std_y = std::sqrt(var_y / n_samples);

    // Y noise should be larger than X noise
    EXPECT_GT(std_y, std_x * 2.0f);
}

// ============================================================================
// IMUMeasurement struct tests
// ============================================================================

TEST(IMUMeasurement, DefaultConstructor) {
    IMUMeasurement meas;
    EXPECT_NEAR(meas.acc.x(), 0.0f, 1e-6f);
    EXPECT_NEAR(meas.acc.y(), 0.0f, 1e-6f);
    EXPECT_NEAR(meas.gyr, 0.0f, 1e-6f);
}

TEST(IMUMeasurement, ConstructorWithVector2f) {
    Eigen::Vector2f acc(1.0f, 2.0f);
    IMUMeasurement meas(acc, 0.5f);

    EXPECT_NEAR(meas.acc.x(), 1.0f, 1e-6f);
    EXPECT_NEAR(meas.acc.y(), 2.0f, 1e-6f);
    EXPECT_NEAR(meas.gyr, 0.5f, 1e-6f);
}

TEST(IMUMeasurement, ConstructorWithFloats) {
    IMUMeasurement meas(1.0f, 2.0f, 0.5f);

    EXPECT_NEAR(meas.acc.x(), 1.0f, 1e-6f);
    EXPECT_NEAR(meas.acc.y(), 2.0f, 1e-6f);
    EXPECT_NEAR(meas.gyr, 0.5f, 1e-6f);
}
