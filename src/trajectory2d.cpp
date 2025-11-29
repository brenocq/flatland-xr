// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include "trajectory2d.hpp"
#include <algorithm>
#include <cmath>

void Trajectory2D::build(const std::vector<Eigen::Vector3f>& poses) {
    _valid = false;
    _num_poses = poses.size();

    if (poses.size() < 2) {
        return;
    }

    // Extract x, y, theta from poses
    Eigen::RowVectorXf x_vals(poses.size());
    Eigen::RowVectorXf y_vals(poses.size());
    std::vector<float> theta_vals_raw(poses.size());

    for (size_t i = 0; i < poses.size(); i++) {
        x_vals(i) = poses[i].x();
        y_vals(i) = poses[i].y();
        theta_vals_raw[i] = poses[i].z();
    }

    // Unwrap angles for continuity
    std::vector<float> theta_unwrapped = unwrap_angles(theta_vals_raw);
    Eigen::RowVectorXf theta_vals(poses.size());
    for (size_t i = 0; i < poses.size(); i++) {
        theta_vals(i) = theta_unwrapped[i];
    }

    // Create parameter values (knots) at t=0,1,2,...,n-1 normalized to [0,1]
    Eigen::RowVectorXf knots(poses.size());
    for (size_t i = 0; i < poses.size(); i++) {
        knots(i) = static_cast<float>(i) / static_cast<float>(poses.size() - 1);
    }

    // Fit cubic splines (degree 3)
    constexpr int degree = 3;
    int actual_degree = std::min(degree, static_cast<int>(poses.size()) - 1);

    _spline_x = Eigen::SplineFitting<Spline1D>::Interpolate(x_vals, actual_degree, knots);
    _spline_y = Eigen::SplineFitting<Spline1D>::Interpolate(y_vals, actual_degree, knots);
    _spline_theta = Eigen::SplineFitting<Spline1D>::Interpolate(theta_vals, actual_degree, knots);

    _valid = true;
}

float Trajectory2D::normalize_param(float t) const {
    if (_num_poses <= 1)
        return 0.0f;
    float u = t / static_cast<float>(_num_poses - 1);
    return std::max(0.0f, std::min(u, 1.0f));
}

std::vector<float> Trajectory2D::unwrap_angles(const std::vector<float>& angles) const {
    if (angles.empty())
        return {};

    std::vector<float> unwrapped(angles.size());
    unwrapped[0] = angles[0];

    for (size_t i = 1; i < angles.size(); i++) {
        float diff = angles[i] - angles[i - 1];
        // Normalize difference to [-pi, pi]
        while (diff > M_PI)
            diff -= 2.0f * M_PI;
        while (diff < -M_PI)
            diff += 2.0f * M_PI;
        unwrapped[i] = unwrapped[i - 1] + diff;
    }

    return unwrapped;
}

Eigen::Vector2f Trajectory2D::position(float t) const {
    if (!_valid)
        return Eigen::Vector2f::Zero();

    float u = normalize_param(t);
    float x = _spline_x(u)(0);
    float y = _spline_y(u)(0);
    return Eigen::Vector2f(x, y);
}

float Trajectory2D::orientation(float t) const {
    if (!_valid)
        return 0.0f;

    float u = normalize_param(t);
    float theta = _spline_theta(u)(0);
    // Normalize to [-pi, pi]
    while (theta > M_PI)
        theta -= 2.0f * M_PI;
    while (theta < -M_PI)
        theta += 2.0f * M_PI;
    return theta;
}

Eigen::Vector3f Trajectory2D::pose(float t) const {
    Eigen::Vector2f pos = position(t);
    float theta = orientation(t);
    return Eigen::Vector3f(pos.x(), pos.y(), theta);
}

Eigen::Vector2f Trajectory2D::velocity(float t) const {
    if (!_valid)
        return Eigen::Vector2f::Zero();

    float u = normalize_param(t);

    // Get spline derivatives (spline returns spline values and derivatives stacked)
    // derivatives(u, 1) returns spline value and first derivative
    auto dx = _spline_x.derivatives(u, 1);
    auto dy = _spline_y.derivatives(u, 1);

    // Scale by chain rule: dS/dt = dS/du * du/dt, where du/dt = 1/(n-1)
    float scale = 1.0f / static_cast<float>(_num_poses - 1);
    return Eigen::Vector2f(dx(1) * scale, dy(1) * scale);
}

float Trajectory2D::angular_velocity(float t) const {
    if (!_valid)
        return 0.0f;

    float u = normalize_param(t);
    auto dtheta = _spline_theta.derivatives(u, 1);

    float scale = 1.0f / static_cast<float>(_num_poses - 1);
    return dtheta(1) * scale;
}

Eigen::Vector2f Trajectory2D::acceleration(float t) const {
    if (!_valid)
        return Eigen::Vector2f::Zero();

    float u = normalize_param(t);

    auto d2x = _spline_x.derivatives(u, 2);
    auto d2y = _spline_y.derivatives(u, 2);

    // Scale by chain rule squared
    float scale = 1.0f / static_cast<float>(_num_poses - 1);
    float scale2 = scale * scale;
    return Eigen::Vector2f(d2x(2) * scale2, d2y(2) * scale2);
}

float Trajectory2D::angular_acceleration(float t) const {
    if (!_valid)
        return 0.0f;

    float u = normalize_param(t);
    auto d2theta = _spline_theta.derivatives(u, 2);

    float scale = 1.0f / static_cast<float>(_num_poses - 1);
    float scale2 = scale * scale;
    return d2theta(2) * scale2;
}

float Trajectory2D::speed(float t) const {
    Eigen::Vector2f vel = velocity(t);
    return vel.norm();
}
