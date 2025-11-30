// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include <cmath>
#include <estimation/imu_integrator.hpp>

namespace estimation {

void IMUIntegrator::set_process_noise(float acc_noise, float gyr_noise) {
    _acc_noise_std = acc_noise;
    _gyr_noise_std = gyr_noise;
}

void IMUIntegrator::reset() {
    _current_state = EstimatedState();
    _state_history.clear();
}

void IMUIntegrator::initialize(const Eigen::Vector3f& initial_pose, const Eigen::Vector2f& initial_vel) {
    _current_state.pose = initial_pose;
    _current_state.velocity = initial_vel;
    _current_state.pose_cov = Eigen::Matrix3f::Zero(); // Perfect initial knowledge
    _current_state.vel_cov = Eigen::Matrix2f::Zero();
    _state_history.clear();
    _state_history.push_back(_current_state);
}

void IMUIntegrator::process_imu(const sensors::IMUMeasurement& imu, float dt) {
    if (!_imu)
        return;

    // Get current state
    float theta = _current_state.pose.z();
    Eigen::Vector2f pos = _current_state.pose.head<2>();
    Eigen::Vector2f vel = _current_state.velocity;

    // Remove bias from IMU measurements
    Eigen::Vector2f acc_body = imu.acc - _imu->acc_bias();
    float gyr = imu.gyr - _imu->gyr_bias();

    // Rotate acceleration from body frame to world frame
    float cos_t = std::cosf(theta);
    float sin_t = std::sinf(theta);
    Eigen::Matrix2f R_bw; // Body to world rotation
    R_bw << cos_t, -sin_t, sin_t, cos_t;
    Eigen::Vector2f acc_world = R_bw * acc_body;

    // Add gravity back (accelerometer measures specific force = acc - gravity)
    acc_world += _gravity;

    // Integrate orientation: theta_new = theta + omega * dt
    float new_theta = theta + gyr * dt;

    // Integrate velocity: v_new = v + a * dt
    Eigen::Vector2f new_vel = vel + acc_world * dt;

    // Integrate position: p_new = p + v * dt + 0.5 * a * dt^2
    Eigen::Vector2f new_pos = pos + vel * dt + 0.5f * acc_world * dt * dt;

    // Update state
    _current_state.pose = Eigen::Vector3f(new_pos.x(), new_pos.y(), new_theta);
    _current_state.velocity = new_vel;

    // Propagate covariance using first-order approximation
    // State: [x, y, theta], Control: [ax_body, ay_body, omega]
    // The state transition Jacobian w.r.t. state
    Eigen::Matrix3f F = Eigen::Matrix3f::Identity();
    F(0, 2) = -acc_body.x() * sin_t * dt * dt * 0.5f - acc_body.y() * cos_t * dt * dt * 0.5f;
    F(1, 2) = acc_body.x() * cos_t * dt * dt * 0.5f - acc_body.y() * sin_t * dt * dt * 0.5f;

    // Process noise covariance (simplified)
    Eigen::Matrix3f Q = Eigen::Matrix3f::Zero();
    float acc_var = _acc_noise_std * _acc_noise_std;
    float gyr_var = _gyr_noise_std * _gyr_noise_std;
    Q(0, 0) = acc_var * dt * dt * dt * dt / 4.0f; // Position noise from acc
    Q(1, 1) = acc_var * dt * dt * dt * dt / 4.0f;
    Q(2, 2) = gyr_var * dt * dt; // Orientation noise from gyro

    _current_state.pose_cov = F * _current_state.pose_cov * F.transpose() + Q;

    // Velocity covariance propagation
    Eigen::Matrix2f Q_vel = Eigen::Matrix2f::Identity() * acc_var * dt * dt;
    _current_state.vel_cov = _current_state.vel_cov + Q_vel;

    // Store in history
    _state_history.push_back(_current_state);
}

void IMUIntegrator::process_camera(const sensors::CameraFrame& frame) {
    // IMU-only estimator ignores camera measurements
    (void)frame;
}

EstimatedState IMUIntegrator::get_state() const { return _current_state; }

EstimationResult IMUIntegrator::get_result() const {
    EstimationResult result;
    result.states = _state_history;
    return result;
}

} // namespace estimation
