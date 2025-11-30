// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#pragma once

#include <Eigen/Dense>
#include <optional>
#include <vector>

namespace core {

/// Check if two line segments intersect
/// Returns the intersection point if they intersect, nullopt otherwise
std::optional<Eigen::Vector2f> segment_intersection(const Eigen::Vector2f& p1, const Eigen::Vector2f& p2, const Eigen::Vector2f& p3,
                                                    const Eigen::Vector2f& p4);

/// Check if a ray from origin in direction intersects a line segment
/// Returns the intersection point if it intersects, nullopt otherwise
std::optional<Eigen::Vector2f> ray_segment_intersection(const Eigen::Vector2f& ray_origin, const Eigen::Vector2f& ray_dir,
                                                        const Eigen::Vector2f& seg_start, const Eigen::Vector2f& seg_end);

/// Compute distance from a point to a line segment
float point_to_segment_distance(const Eigen::Vector2f& point, const Eigen::Vector2f& seg_start, const Eigen::Vector2f& seg_end);

/// Normalize angle to [-pi, pi]
float normalize_angle(float angle);

/// Compute angle difference (shortest path), result in [-pi, pi]
float angle_difference(float from, float to);

/// Unwrap angles to ensure continuity (for spline fitting)
std::vector<float> unwrap_angles(const std::vector<float>& angles);

} // namespace core
