// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#pragma once

#include <Eigen/Dense>
#include <core/types.hpp>
#include <sensors/sensor_data.hpp>
#include <vector>

namespace estimation {

/// Estimated state with covariance at a single time step
struct EstimatedState {
    Eigen::Vector3f pose = Eigen::Vector3f::Zero();         ///< (x, y, theta)
    Eigen::Vector2f velocity = Eigen::Vector2f::Zero();     ///< (vx, vy)
    Eigen::Matrix3f pose_cov = Eigen::Matrix3f::Identity(); ///< 3x3 covariance of (x, y, theta)
    Eigen::Matrix2f vel_cov = Eigen::Matrix2f::Identity();  ///< 2x2 covariance of (vx, vy)

    EstimatedState() = default;
    EstimatedState(const Eigen::Vector3f& p, const Eigen::Vector2f& v) : pose(p), velocity(v) {}
};

/// Result of running an estimator
struct EstimationResult {
    std::vector<EstimatedState> states; ///< Estimated states at each time step

    bool is_valid() const { return !states.empty(); }
    size_t num_steps() const { return states.size(); }

    void clear() { states.clear(); }

    /// Extract pose vectors for plotting
    std::vector<Eigen::Vector3f> get_poses() const {
        std::vector<Eigen::Vector3f> poses;
        poses.reserve(states.size());
        for (const auto& s : states) {
            poses.push_back(s.pose);
        }
        return poses;
    }

    /// Extract velocity vectors for plotting
    std::vector<Eigen::Vector2f> get_velocities() const {
        std::vector<Eigen::Vector2f> vels;
        vels.reserve(states.size());
        for (const auto& s : states) {
            vels.push_back(s.velocity);
        }
        return vels;
    }
};

/// Base class for state estimators
class EstimatorBase {
  public:
    virtual ~EstimatorBase() = default;

    /// Reset estimator to initial state
    virtual void reset() = 0;

    /// Initialize with a known initial state
    virtual void initialize(const Eigen::Vector3f& initial_pose, const Eigen::Vector2f& initial_vel) = 0;

    /// Process an IMU measurement
    virtual void process_imu(const sensors::IMUMeasurement& imu, float dt) = 0;

    /// Process a camera measurement
    virtual void process_camera(const sensors::CameraFrame& frame) = 0;

    /// Get the current estimated state
    virtual EstimatedState get_state() const = 0;

    /// Get full estimation result (all states)
    virtual EstimationResult get_result() const = 0;
};

} // namespace estimation
