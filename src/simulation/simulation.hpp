// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#pragma once

#include <Eigen/Dense>
#include <core/trajectory2d.hpp>
#include <core/types.hpp>
#include <sensors/camera2d.hpp>
#include <sensors/imu2d.hpp>
#include <vector>

namespace simulation {

/// Result of running a simulation
struct SimulationResult {
    // Ground truth states at each time step
    std::vector<Eigen::Vector3f> gt_poses; ///< Ground truth poses (x, y, theta)
    std::vector<Eigen::Vector2f> gt_vel;   ///< Ground truth velocities (vx, vy)
    std::vector<Eigen::Vector2f> gt_acc;   ///< Ground truth accelerations (ax, ay)
    std::vector<float> gt_omega;           ///< Ground truth angular velocities

    // Ground truth measurements (no noise)
    std::vector<sensors::IMUMeasurement> gt_imu;
    std::vector<std::vector<sensors::LandmarkObservation>> gt_cam;

    // Noisy measurements
    std::vector<sensors::IMUMeasurement> imu_measurements;
    std::vector<std::vector<sensors::LandmarkObservation>> cam_measurements;

    /// Check if simulation result is valid (has data)
    bool is_valid() const { return !gt_poses.empty(); }

    /// Get number of time steps
    size_t num_steps() const { return gt_poses.size(); }

    /// Clear all simulation data
    void clear();
};

/// Configuration for the simulation
struct SimulationConfig {
    Eigen::Vector2f gravity = Eigen::Vector2f::Zero(); ///< World gravity vector
};

/// Check if a ray from camera to landmark is blocked by any wall
bool is_landmark_occluded(const Eigen::Vector2f& camera_pos, const Eigen::Vector2f& landmark, const std::vector<core::Wall>& walls);

/// Run simulation given world data and sensor models
/// @param trajectory Ground truth trajectory to sample from
/// @param landmarks World landmarks
/// @param walls World walls for occlusion
/// @param camera Camera sensor model
/// @param imu IMU sensor model
/// @param config Simulation configuration
/// @return Simulation result with ground truth and measurements
SimulationResult run(const core::Trajectory2D& trajectory, const std::vector<Eigen::Vector2f>& landmarks, const std::vector<core::Wall>& walls,
                     sensors::Camera2D& camera, sensors::IMU2D& imu, const SimulationConfig& config = {});

} // namespace simulation
