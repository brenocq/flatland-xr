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
    // Small initial covariance (not perfect knowledge)
    _current_state.pose_cov = Eigen::Matrix3f::Identity() * 0.01f; // 10cm std for position, 0.01 rad for orientation
    _current_state.vel_cov = Eigen::Matrix2f::Identity() * 0.01f;  // 10cm/s std for velocity
    _state_history.clear();
    _state_history.push_back(_current_state);
}

void IMUIntegrator::process_imu(const sensors::IMUMeasurement& imu, float dt) {
    if (!_imu)
        return;

    // State vector indices: [x, y, theta, vx, vy]
    constexpr size_t STATE_DIM = 5;
    constexpr size_t IDX_X = 0;
    constexpr size_t IDX_Y = 1;
    constexpr size_t IDX_THETA = 2;
    constexpr size_t IDX_VX = 3;
    constexpr size_t IDX_VY = 4;

    // Get current state
    float theta = _current_state.pose.z();
    Eigen::Vector2f pos = _current_state.pose.head<2>();
    Eigen::Vector2f vel = _current_state.velocity;

    // Remove bias from IMU measurements
    Eigen::Vector2f acc_body = imu.acc - _imu->acc_bias();
    float gyr = imu.gyr - _imu->gyr_bias();

    // Rotate acceleration from body frame to world frame
    float cos_t = std::cos(theta);
    float sin_t = std::sin(theta);
    Eigen::Matrix2f R_bw; // Body to world rotation
    R_bw << cos_t, -sin_t, sin_t, cos_t;
    Eigen::Vector2f acc_world = R_bw * acc_body;

    // Add gravity back (accelerometer measures specific force = acc - gravity)
    acc_world += _gravity;

    // State propagation (prediction step)
    // x_new = x + vx * dt
    // y_new = y + vy * dt
    // theta_new = theta + omega * dt
    // vx_new = vx + ax_world * dt
    // vy_new = vy + ay_world * dt

    float new_theta = theta + gyr * dt;
    Eigen::Vector2f new_vel = vel + acc_world * dt;
    Eigen::Vector2f new_pos = pos + vel * dt;

    // Update state
    _current_state.pose = Eigen::Vector3f(new_pos.x(), new_pos.y(), new_theta);
    _current_state.velocity = new_vel;

    // Build the state transition matrix F (5x5)
    // F is mostly identity except for coupling terms
    Eigen::Matrix<float, STATE_DIM, STATE_DIM> F = Eigen::Matrix<float, STATE_DIM, STATE_DIM>::Identity();

    // Position depends on velocity
    F(IDX_X, IDX_VX) = dt;
    F(IDX_Y, IDX_VY) = dt;

    // Velocity depends on orientation (through rotation of body frame acceleration)
    F(IDX_VX, IDX_THETA) = (-acc_body.x() * sin_t - acc_body.y() * cos_t) * dt;
    F(IDX_VY, IDX_THETA) = (acc_body.x() * cos_t - acc_body.y() * sin_t) * dt;

    // Build the process noise covariance Q (5x5)
    // Noise comes from: acceleration noise (affects vx, vy) and gyroscope noise (affects theta)
    Eigen::Matrix<float, STATE_DIM, STATE_DIM> Q = Eigen::Matrix<float, STATE_DIM, STATE_DIM>::Zero();

    float acc_var = _acc_noise_std * _acc_noise_std;
    float gyr_var = _gyr_noise_std * _gyr_noise_std;

    // Position noise (from velocity uncertainty propagation: integral of velocity noise)
    Q(IDX_X, IDX_X) = acc_var * dt * dt * dt * dt / 4.0f;
    Q(IDX_Y, IDX_Y) = acc_var * dt * dt * dt * dt / 4.0f;

    // Orientation noise (from gyroscope)
    Q(IDX_THETA, IDX_THETA) = gyr_var * dt * dt;

    // Velocity noise (from accelerometer)
    Q(IDX_VX, IDX_VX) = acc_var * dt * dt;
    Q(IDX_VY, IDX_VY) = acc_var * dt * dt;

    // Build combined covariance matrix (5x5)
    Eigen::Matrix<float, STATE_DIM, STATE_DIM> P = Eigen::Matrix<float, STATE_DIM, STATE_DIM>::Zero();
    P.block<3, 3>(IDX_X, IDX_X) = _current_state.pose_cov;
    P.block<2, 2>(IDX_VX, IDX_VX) = _current_state.vel_cov;
    // Cross-covariances between pose and velocity will be propagated

    // Covariance propagation: P_new = F * P * F^T + Q
    P = F * P * F.transpose() + Q;

    // Extract pose and velocity covariances
    _current_state.pose_cov = P.block<3, 3>(IDX_X, IDX_X);
    _current_state.vel_cov = P.block<2, 2>(IDX_VX, IDX_VX);

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
