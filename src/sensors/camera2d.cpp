// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <sensors/camera2d.hpp>

namespace sensors {

void Camera2D::set_intrinsics(int width, float fov) {
    _width = width;
    _principal_point = static_cast<float>(width) / 2.0f;
    // fov = 2 * atan(width / (2 * fx))  =>  fx = width / (2 * tan(fov/2))
    _focal_length = static_cast<float>(width) / (2.0f * std::tan(fov / 2.0f));
}

void Camera2D::set_width(int width) {
    // Keep FOV constant by scaling focal_length proportionally
    float current_fov = fov();
    _width = width;
    _principal_point = static_cast<float>(width) / 2.0f;
    _focal_length = static_cast<float>(width) / (2.0f * std::tan(current_fov / 2.0f));
}

void Camera2D::set_fov(float fov_rad) {
    // Adjust focal_length to achieve desired FOV
    _focal_length = static_cast<float>(_width) / (2.0f * std::tan(fov_rad / 2.0f));
}

void Camera2D::set_noise_std(float noise_std) { _noise_std = noise_std; }

float Camera2D::fov() const {
    // fov = 2 * atan(width / (2 * fx))
    return 2.0f * std::atan(static_cast<float>(_width) / (2.0f * _focal_length));
}

float Camera2D::distance_point_segment(const Eigen::Vector2f& p, const Eigen::Vector2f& a, const Eigen::Vector2f& b) const {
    Eigen::Vector2f ab = b - a;
    float ab_len2 = ab.squaredNorm();
    if (ab_len2 <= std::numeric_limits<float>::epsilon()) {
        return (p - a).norm();
    }
    float t = (p - a).dot(ab) / ab_len2;
    t = std::clamp(t, 0.0f, 1.0f);
    Eigen::Vector2f proj = a + t * ab;
    return (p - proj).norm();
}

float Camera2D::signed_distance_to_walls(const Eigen::Vector2f& p, const std::vector<core::Wall>& walls) const {
    float min_dist = std::numeric_limits<float>::max();
    for (const auto& wall : walls) {
        for (size_t i = 0; i + 1 < wall.points.size(); ++i) {
            float d = distance_point_segment(p, wall.points[i], wall.points[i + 1]);
            min_dist = std::min(min_dist, d);
        }
    }
    return min_dist;
}

core::RayHit Camera2D::find_wall_color_at_hit(const Eigen::Vector2f& p, const std::vector<core::Wall>& walls) const {
    float min_dist = std::numeric_limits<float>::max();
    core::RayHit hit;
    hit.hit_pos = p;

    for (const auto& wall : walls) {
        for (size_t i = 0; i + 1 < wall.points.size(); ++i) {
            float d = distance_point_segment(p, wall.points[i], wall.points[i + 1]);
            if (d < min_dist) {
                min_dist = d;
                hit.color = wall.color;
                hit.hit = true;
            }
        }
    }

    return hit;
}

core::RayHit Camera2D::march_ray(const Eigen::Vector2f& origin, const Eigen::Vector2f& dir, const std::vector<core::Wall>& walls) const {
    const float max_dist = 50.0f;
    const int max_steps = 128;
    const float hit_threshold = 0.01f;

    float t = 0.0f;
    for (int i = 0; i < max_steps && t < max_dist; ++i) {
        Eigen::Vector2f p = origin + t * dir;

        float min_d = signed_distance_to_walls(p, walls);

        if (min_d < hit_threshold) {
            core::RayHit hit = find_wall_color_at_hit(p, walls);
            if (!hit.hit) {
                hit.hit = true;
                hit.hit_pos = p;
            }
            return hit;
        }

        if (!std::isfinite(min_d) || min_d <= 0.0f) {
            min_d = 0.01f;
        }
        t += min_d;
    }

    return core::RayHit{};
}

std::optional<float> Camera2D::project(const Eigen::Vector3f& pose, const Eigen::Vector2f& landmark) const {
    // Camera position and orientation
    Eigen::Vector2f cam_pos(pose.x(), pose.y());
    float cam_angle = pose.z();

    // Vector from camera to landmark in world frame
    Eigen::Vector2f to_landmark = landmark - cam_pos;

    // Build rotation matrix (world to camera)
    float cos_a = std::cos(cam_angle);
    float sin_a = std::sin(cam_angle);
    Eigen::Matrix2f R_wc;
    R_wc << cos_a, sin_a, -sin_a, cos_a;

    // Transform to camera frame (homogeneous-style)
    // In camera frame: x is forward, y is left
    Eigen::Vector2f p_cam = R_wc * to_landmark;
    float x_cam = p_cam.x();
    float y_cam = p_cam.y();

    // Check if landmark is in front of camera
    if (x_cam <= 0) {
        return std::nullopt;
    }

    // Project using pinhole model with homogeneous coordinates
    // Normalized coordinates: y_norm = y_cam / x_cam
    // Pixel coordinates: u = -fx * y_norm + cx
    // (negative because y_cam positive is left, but u increases to the right)
    float u = -_focal_length * (y_cam / x_cam) + _principal_point;

    // Check if within image bounds
    if (u < 0 || u > static_cast<float>(_width)) {
        return std::nullopt;
    }

    return u;
}

Image1D Camera2D::raytrace(const Eigen::Vector3f& pose, const std::vector<core::Wall>& walls) const {
    Image1D scanline;
    scanline.rays.resize(static_cast<size_t>(_width), core::RayHit{});

    Eigen::Vector2f cam_pos(pose.x(), pose.y());
    float cam_angle = pose.z();
    float fx = _focal_length;
    float cx = _principal_point;

    float cos_a = std::cos(cam_angle);
    float sin_a = std::sin(cam_angle);

    Eigen::Vector2f nan_vec(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN());

    for (int px = 0; px < _width; ++px) {
        float alpha = std::atan((static_cast<float>(px) - cx) / fx);

        float dir_cam_x = std::cos(alpha);
        float dir_cam_y = -std::sin(alpha);

        Eigen::Vector2f dir_world;
        dir_world.x() = cos_a * dir_cam_x - sin_a * dir_cam_y;
        dir_world.y() = sin_a * dir_cam_x + cos_a * dir_cam_y;
        dir_world.normalize();

        core::RayHit hit = march_ray(cam_pos, dir_world, walls);
        size_t pixel_idx = static_cast<size_t>(px);
        scanline.rays[pixel_idx] = hit;
    }

    return scanline;
}



std::vector<CameraMeasurement> Camera2D::project_landmarks(const Eigen::Vector3f& pose, const std::vector<Eigen::Vector2f>& landmarks) const {
    std::vector<CameraMeasurement> observations;
    observations.reserve(landmarks.size());

    for (size_t i = 0; i < landmarks.size(); ++i) {
        auto u = project(pose, landmarks[i]);
        if (u.has_value()) {
            observations.emplace_back(u.value(), i);
        }
    }

    return observations;
}

std::optional<float> Camera2D::measure(const Eigen::Vector3f& pose, const Eigen::Vector2f& landmark) {
    auto u = project(pose, landmark);
    if (!u.has_value()) {
        return std::nullopt;
    }

    float noisy_u = u.value();

    // Add Gaussian noise only if noise_std > 0
    if (_noise_std > 0.0f) {
        std::normal_distribution<float> noise(0.0f, _noise_std);
        noisy_u += noise(_rng);
    }

    // Clamp to image bounds
    noisy_u = std::clamp(noisy_u, 0.0f, static_cast<float>(_width));

    return noisy_u;
}

std::vector<CameraMeasurement> Camera2D::measure_landmarks(const Eigen::Vector3f& pose, const std::vector<Eigen::Vector2f>& landmarks) {
    std::vector<CameraMeasurement> observations;
    observations.reserve(landmarks.size());

    for (size_t i = 0; i < landmarks.size(); ++i) {
        auto u = measure(pose, landmarks[i]);
        if (u.has_value()) {
            observations.emplace_back(u.value(), i);
        }
    }

    return observations;
}

} // namespace sensors
