// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include "imu2d.hpp"

namespace sensors {

void IMU2D::set_acc_bias(const Eigen::Vector2f& bias) { _acc_bias = bias; }

void IMU2D::set_gyr_bias(float bias) { _gyr_bias = bias; }

void IMU2D::set_acc_noise_std(const Eigen::Vector2f& noise_std) { _acc_noise_std = noise_std; }

void IMU2D::set_gyr_noise_std(float noise_std) { _gyr_noise_std = noise_std; }

IMUMeasurement IMU2D::measure(const Eigen::Vector2f& gt_acc, float gt_gyr) {
    std::normal_distribution<float> acc_noise_x(0.0f, _acc_noise_std.x());
    std::normal_distribution<float> acc_noise_y(0.0f, _acc_noise_std.y());
    std::normal_distribution<float> gyr_noise(0.0f, _gyr_noise_std);

    IMUMeasurement meas;
    meas.acc = gt_acc + _acc_bias + Eigen::Vector2f(acc_noise_x(_rng), acc_noise_y(_rng));
    meas.gyr = gt_gyr + _gyr_bias + gyr_noise(_rng);

    return meas;
}

} // namespace sensors
