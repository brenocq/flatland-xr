// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include "camera2d.hpp"
#include <cmath>

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

std::vector<LandmarkObservation> Camera2D::project_landmarks(const Eigen::Vector3f& pose, const std::vector<Eigen::Vector2f>& landmarks) const {
    std::vector<LandmarkObservation> observations;
    observations.reserve(landmarks.size());

    for (size_t i = 0; i < landmarks.size(); i++) {
        auto u = project(pose, landmarks[i]);
        if (u.has_value()) {
            observations.push_back(LandmarkObservation(u.value(), i));
        }
    }

    return observations;
}

std::optional<float> Camera2D::measure(const Eigen::Vector3f& pose, const Eigen::Vector2f& landmark) {
    auto u = project(pose, landmark);
    if (!u.has_value()) {
        return std::nullopt;
    }

    // Add Gaussian noise
    std::normal_distribution<float> noise(0.0f, _noise_std);
    float noisy_u = u.value() + noise(_rng);

    // Clamp to image bounds
    noisy_u = std::clamp(noisy_u, 0.0f, static_cast<float>(_width));

    return noisy_u;
}

std::vector<LandmarkObservation> Camera2D::measure_landmarks(const Eigen::Vector3f& pose, const std::vector<Eigen::Vector2f>& landmarks) {
    std::vector<LandmarkObservation> observations;
    observations.reserve(landmarks.size());

    for (size_t i = 0; i < landmarks.size(); i++) {
        auto u = measure(pose, landmarks[i]);
        if (u.has_value()) {
            observations.push_back(LandmarkObservation(u.value(), i));
        }
    }

    return observations;
}

} // namespace sensors
