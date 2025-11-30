// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#pragma once

#include "sensor_data.hpp"
#include <Eigen/Dense>
#include <random>

namespace sensors {

/// 2D IMU model storing intrinsic parameters (biases and noise)
class IMU2D {
  public:
    IMU2D() = default;

    /// Set accelerometer bias
    /// @param bias 2D accelerometer bias (x, y) in m/s²
    void set_acc_bias(const Eigen::Vector2f& bias);

    /// Set gyroscope bias
    /// @param bias Gyroscope bias in rad/s
    void set_gyr_bias(float bias);

    /// Set accelerometer noise standard deviation
    /// @param noise_std Accelerometer noise std (x, y) in m/s²
    void set_acc_noise_std(const Eigen::Vector2f& noise_std);

    /// Set gyroscope noise standard deviation
    /// @param noise_std Gyroscope noise std in rad/s
    void set_gyr_noise_std(float noise_std);

    /// Get accelerometer bias
    Eigen::Vector2f acc_bias() const { return _acc_bias; }

    /// Get gyroscope bias
    float gyr_bias() const { return _gyr_bias; }

    /// Get accelerometer noise standard deviation
    Eigen::Vector2f acc_noise_std() const { return _acc_noise_std; }

    /// Get gyroscope noise standard deviation
    float gyr_noise_std() const { return _gyr_noise_std; }

    /// Generate noisy IMU measurement from ground truth acceleration and angular velocity
    /// @param gt_acc Ground truth acceleration in body frame
    /// @param gt_gyr Ground truth angular velocity (rad/s)
    /// @return Noisy IMU measurement with bias and Gaussian noise added
    IMUMeasurement measure(const Eigen::Vector2f& gt_acc, float gt_gyr);

  private:
    Eigen::Vector2f _acc_bias = Eigen::Vector2f::Zero();            ///< Accelerometer bias (x, y)
    float _gyr_bias = 0.0f;                                         ///< Gyroscope bias in rad/s
    Eigen::Vector2f _acc_noise_std = Eigen::Vector2f(0.01f, 0.01f); ///< Accelerometer noise std
    float _gyr_noise_std = 0.001f;                                  ///< Gyroscope noise std in rad/s

    std::mt19937 _rng{std::random_device{}()}; ///< Random number generator
};

} // namespace sensors
