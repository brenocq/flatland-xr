// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#pragma once

#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>
#include <vector>

/// Continuous 2D trajectory using cubic splines
/// Parameterized by arc-length so distance between integer parameters is constant
class Trajectory2D {
  public:
    Trajectory2D() = default;

    /// Build trajectory from discrete poses (x, y, theta)
    /// Poses are assumed to be at t=0,1,2,3,...
    void build(const std::vector<Eigen::Vector3f>& poses);

    /// Check if trajectory has been built
    bool is_valid() const { return _valid; }

    /// Get number of poses used to build the trajectory
    size_t num_poses() const { return _num_poses; }

    /// Get position at parameter t (can be fractional)
    Eigen::Vector2f position(float t) const;

    /// Get orientation at parameter t (can be fractional)
    float orientation(float t) const;

    /// Get full pose (x, y, theta) at parameter t
    Eigen::Vector3f pose(float t) const;

    /// Get linear velocity (dx/dt, dy/dt) at parameter t
    Eigen::Vector2f velocity(float t) const;

    /// Get angular velocity (dtheta/dt) at parameter t
    float angular_velocity(float t) const;

    /// Get linear acceleration (d²x/dt², d²y/dt²) at parameter t
    Eigen::Vector2f acceleration(float t) const;

    /// Get angular acceleration (d²theta/dt²) at parameter t
    float angular_acceleration(float t) const;

    /// Get speed (magnitude of velocity) at parameter t
    float speed(float t) const;

    /// Get total arc-length of the trajectory
    float total_length() const { return _valid ? static_cast<float>(_num_poses - 1) : 0.0f; }

    /// Get maximum valid parameter value
    float max_t() const { return _valid ? static_cast<float>(_num_poses - 1) : 0.0f; }

  private:
    /// Normalize parameter t to [0, 1] for spline evaluation
    float normalize_param(float t) const;

    /// Unwrap angles to ensure continuity for spline fitting
    std::vector<float> unwrap_angles(const std::vector<float>& angles) const;

    bool _valid = false;
    size_t _num_poses = 0;

    // Splines for x(t), y(t), theta(t)
    using Spline1D = Eigen::Spline<float, 1>;
    Spline1D _spline_x;
    Spline1D _spline_y;
    Spline1D _spline_theta;
};
