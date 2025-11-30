// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include "simulation.hpp"
#include <core/geometry.hpp>

namespace simulation {

void SimulationResult::clear() {
    gt_poses.clear();
    gt_vel.clear();
    gt_acc.clear();
    gt_omega.clear();
    gt_imu.clear();
    gt_cam.clear();
    imu_measurements.clear();
    cam_measurements.clear();
}

bool is_landmark_occluded(const Eigen::Vector2f& camera_pos, const Eigen::Vector2f& landmark, const std::vector<core::Wall>& walls) {
    for (const auto& wall : walls) {
        for (size_t i = 0; i + 1 < wall.points.size(); i++) {
            if (core::segments_intersect(camera_pos, landmark, wall.points[i], wall.points[i + 1])) {
                return true;
            }
        }
    }
    return false;
}

SimulationResult run(const core::Trajectory2D& trajectory, const std::vector<Eigen::Vector2f>& landmarks, const std::vector<core::Wall>& walls,
                     sensors::Camera2D& camera, sensors::IMU2D& imu, const SimulationConfig& config) {
    SimulationResult result;

    if (!trajectory.is_valid())
        return result;

    // Sample ground truth states from trajectory
    size_t num_poses = trajectory.num_poses();
    result.gt_poses.reserve(num_poses);
    result.gt_vel.reserve(num_poses);
    result.gt_acc.reserve(num_poses);
    result.gt_omega.reserve(num_poses);

    for (size_t i = 0; i < num_poses; i++) {
        float t = static_cast<float>(i);
        Eigen::Vector3f pose = trajectory.pose_vector(t);
        result.gt_poses.push_back(pose);

        // Use trajectory methods for velocity/acceleration
        Eigen::Vector2f vel = trajectory.velocity(t);
        float omega = trajectory.angular_velocity(t);
        Eigen::Vector2f acc = trajectory.acceleration(t);

        result.gt_vel.push_back(vel);
        result.gt_omega.push_back(omega);
        result.gt_acc.push_back(acc);
    }

    // Generate sensor measurements
    result.gt_imu.reserve(num_poses);
    result.gt_cam.reserve(num_poses);
    result.imu_measurements.reserve(num_poses);
    result.cam_measurements.reserve(num_poses);

    for (size_t i = 0; i < num_poses; i++) {
        const Eigen::Vector3f& pose = result.gt_poses[i];
        Eigen::Vector2f pos(pose.x(), pose.y());
        float theta = pose.z();

        // IMU measurement
        // The accelerometer measures specific force (acceleration - gravity) in body frame
        // In body frame: a_body = R^T * (a_world - g)
        Eigen::Vector2f world_acc = result.gt_acc[i];
        Eigen::Vector2f specific_force = world_acc - config.gravity;

        // Rotate to body frame
        float cos_t = std::cos(theta);
        float sin_t = std::sin(theta);
        Eigen::Matrix2f R_wb; // World to body rotation
        R_wb << cos_t, sin_t, -sin_t, cos_t;
        Eigen::Vector2f body_acc = R_wb * specific_force;

        float gt_gyr = result.gt_omega[i];

        // Store ground truth IMU (no noise, but with bias for comparison)
        result.gt_imu.push_back({body_acc, gt_gyr});
        // Store noisy measurement
        result.imu_measurements.push_back(imu.measure(body_acc, gt_gyr));

        // Camera measurements (filter landmarks by wall occlusion)
        std::vector<sensors::CameraMeasurement> gt_frame_obs;
        std::vector<sensors::CameraMeasurement> noisy_frame_obs;
        for (size_t j = 0; j < landmarks.size(); j++) {
            if (!is_landmark_occluded(pos, landmarks[j], walls)) {
                auto gt_u = camera.project(pose, landmarks[j]);
                if (gt_u.has_value()) {
                    gt_frame_obs.push_back({gt_u.value(), j});
                    auto noisy_u = camera.measure(pose, landmarks[j]);
                    if (noisy_u.has_value()) {
                        noisy_frame_obs.push_back({noisy_u.value(), j});
                    }
                }
            }
        }
        result.gt_cam.push_back(gt_frame_obs);
        result.cam_measurements.push_back(noisy_frame_obs);
    }

    return result;
}

} // namespace simulation
