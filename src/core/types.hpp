// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#pragma once

#include <Eigen/Dense>
#include <vector>

namespace core {

/// 2D pose (x, y, theta)
struct Pose2D {
    Eigen::Vector2f position = Eigen::Vector2f::Zero();
    float orientation = 0.0f;

    Pose2D() = default;
    Pose2D(float x, float y, float theta) : position(x, y), orientation(theta) {}
    Pose2D(const Eigen::Vector2f& pos, float theta) : position(pos), orientation(theta) {}
    explicit Pose2D(const Eigen::Vector3f& v) : position(v.x(), v.y()), orientation(v.z()) {}

    float x() const { return position.x(); }
    float y() const { return position.y(); }
    float theta() const { return orientation; }

    Eigen::Vector3f to_vector() const { return Eigen::Vector3f(position.x(), position.y(), orientation); }

    /// Rotation matrix from body to world frame
    Eigen::Matrix2f R_body_to_world() const {
        float c = std::cos(orientation);
        float s = std::sin(orientation);
        Eigen::Matrix2f R;
        R << c, -s, s, c;
        return R;
    }

    /// Rotation matrix from world to body frame
    Eigen::Matrix2f R_world_to_body() const { return R_body_to_world().transpose(); }
};

/// 2D velocity (linear + angular)
struct Velocity2D {
    Eigen::Vector2f linear = Eigen::Vector2f::Zero();
    float angular = 0.0f;

    Velocity2D() = default;
    Velocity2D(float vx, float vy, float omega) : linear(vx, vy), angular(omega) {}
    Velocity2D(const Eigen::Vector2f& v, float omega) : linear(v), angular(omega) {}

    float vx() const { return linear.x(); }
    float vy() const { return linear.y(); }
    float omega() const { return angular; }
};

/// 2D acceleration (linear + angular)
struct Acceleration2D {
    Eigen::Vector2f linear = Eigen::Vector2f::Zero();
    float angular = 0.0f;

    Acceleration2D() = default;
    Acceleration2D(float ax, float ay, float alpha) : linear(ax, ay), angular(alpha) {}
    Acceleration2D(const Eigen::Vector2f& a, float alpha) : linear(a), angular(alpha) {}

    float ax() const { return linear.x(); }
    float ay() const { return linear.y(); }
    float alpha() const { return angular; }
};

/// Complete 2D state (pose + velocity)
struct State2D {
    Pose2D pose;
    Velocity2D velocity;

    State2D() = default;
    State2D(const Pose2D& p, const Velocity2D& v) : pose(p), velocity(v) {}
};

/// 2D landmark (point feature in world frame)
struct Landmark {
    Eigen::Vector2f position = Eigen::Vector2f::Zero();
    size_t id = 0;

    Landmark() = default;
    Landmark(float x, float y, size_t id = 0) : position(x, y), id(id) {}
    Landmark(const Eigen::Vector2f& pos, size_t id = 0) : position(pos), id(id) {}

    float x() const { return position.x(); }
    float y() const { return position.y(); }
};

struct RayHit {
    Eigen::Vector3f color = Eigen::Vector3f::Zero();
    Eigen::Vector2f hit_pos = Eigen::Vector2f::Zero();
    bool hit = false;
};

/// Wall represented as a polyline (sequence of connected line segments)
struct Wall {
    std::vector<Eigen::Vector2f> points;                       ///< Points defining the wall polyline
    Eigen::Vector3f color = Eigen::Vector3f(1.0f, 1.0f, 1.0f); ///< Color of the wall (RGB)

    Wall() = default;
    Wall(const std::vector<Eigen::Vector2f>& pts) : points(pts) {}

    bool empty() const { return points.empty(); }
    size_t num_segments() const { return points.size() > 1 ? points.size() - 1 : 0; }
};

} // namespace core
