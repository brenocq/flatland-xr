// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include "geometry.hpp"
#include <cmath>

namespace core {

std::optional<Eigen::Vector2f> segment_intersection(const Eigen::Vector2f& p1, const Eigen::Vector2f& p2, const Eigen::Vector2f& p3,
                                                    const Eigen::Vector2f& p4) {
    Eigen::Vector2f d1 = p2 - p1;
    Eigen::Vector2f d2 = p4 - p3;

    float cross = d1.x() * d2.y() - d1.y() * d2.x();
    if (std::abs(cross) < 1e-10f) {
        return std::nullopt; // Parallel or collinear
    }

    Eigen::Vector2f d3 = p3 - p1;
    float t = (d3.x() * d2.y() - d3.y() * d2.x()) / cross;
    float u = (d3.x() * d1.y() - d3.y() * d1.x()) / cross;

    if (t >= 0.0f && t <= 1.0f && u >= 0.0f && u <= 1.0f) {
        return p1 + t * d1;
    }

    return std::nullopt;
}

std::optional<Eigen::Vector2f> ray_segment_intersection(const Eigen::Vector2f& ray_origin, const Eigen::Vector2f& ray_dir,
                                                        const Eigen::Vector2f& seg_start, const Eigen::Vector2f& seg_end) {
    Eigen::Vector2f seg_dir = seg_end - seg_start;

    float cross = ray_dir.x() * seg_dir.y() - ray_dir.y() * seg_dir.x();
    if (std::abs(cross) < 1e-10f) {
        return std::nullopt; // Parallel
    }

    Eigen::Vector2f d = seg_start - ray_origin;
    float t = (d.x() * seg_dir.y() - d.y() * seg_dir.x()) / cross;
    float u = (d.x() * ray_dir.y() - d.y() * ray_dir.x()) / cross;

    if (t >= 0.0f && u >= 0.0f && u <= 1.0f) {
        return ray_origin + t * ray_dir;
    }

    return std::nullopt;
}

float point_to_segment_distance(const Eigen::Vector2f& point, const Eigen::Vector2f& seg_start, const Eigen::Vector2f& seg_end) {
    Eigen::Vector2f seg = seg_end - seg_start;
    float len_sq = seg.squaredNorm();

    if (len_sq < 1e-10f) {
        return (point - seg_start).norm();
    }

    float t = std::clamp((point - seg_start).dot(seg) / len_sq, 0.0f, 1.0f);
    Eigen::Vector2f projection = seg_start + t * seg;
    return (point - projection).norm();
}

float normalize_angle(float angle) {
    while (angle > M_PI)
        angle -= 2.0f * M_PI;
    while (angle < -M_PI)
        angle += 2.0f * M_PI;
    return angle;
}

float angle_difference(float from, float to) {
    float diff = to - from;
    return normalize_angle(diff);
}

std::vector<float> unwrap_angles(const std::vector<float>& angles) {
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

} // namespace core
