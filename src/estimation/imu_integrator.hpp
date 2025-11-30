// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#pragma once

#include <Eigen/Dense>
#include <estimation/estimator_base.hpp>
#include <memory>
#include <sensors/imu2d.hpp>

namespace estimation {

/// Simple IMU-only estimator that integrates accelerometer and gyroscope
/// measurements. This estimator will drift over time as there are no
/// corrections from other sensors.
class IMUIntegrator : public EstimatorBase {
  public:
    IMUIntegrator() = default;

    /// Set the IMU model (for bias and noise parameters)
    void set_imu_model(std::shared_ptr<sensors::IMU2D> imu) { _imu = imu; }

    /// Set the gravity vector (world frame, points toward ground)
    void set_gravity(const Eigen::Vector2f& gravity) { _gravity = gravity; }

    /// Set process noise standard deviations
    void set_process_noise(float acc_noise, float gyr_noise);

    void reset() override;
    void initialize(const Eigen::Vector3f& initial_pose, const Eigen::Vector2f& initial_vel) override;
    void process_imu(const sensors::IMUMeasurement& imu, float dt) override;
    void process_camera(const sensors::CameraFrame& frame) override;
    EstimatedState get_state() const override;
    EstimationResult get_result() const override;

  private:
    std::shared_ptr<sensors::IMU2D> _imu;
    Eigen::Vector2f _gravity = Eigen::Vector2f::Zero();

    // Current state
    EstimatedState _current_state;

    // Process noise (standard deviations)
    float _acc_noise_std = 0.1f;
    float _gyr_noise_std = 0.01f;

    // History of all states
    std::vector<EstimatedState> _state_history;
};

} // namespace estimation
