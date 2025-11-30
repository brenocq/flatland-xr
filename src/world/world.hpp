// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#pragma once

#include <core/trajectory2d.hpp>
#include <core/types.hpp>
#include <string>
#include <vector>

namespace world {

/// World preset identifiers
enum class Preset { Custom = 0, ASquaresHouse, VisitFromSphere, HallOfCouncil, COUNT };

/// Get human-readable name for a preset
const char* preset_name(Preset preset);

/// World data containing all static elements
struct World {
    std::vector<Eigen::Vector2f> landmarks;
    std::vector<core::Wall> walls;
    std::vector<Eigen::Vector3f> trajectory_keypoints; ///< Keypoints for trajectory generation

    void clear() {
        landmarks.clear();
        walls.clear();
        trajectory_keypoints.clear();
    }

    bool empty() const { return landmarks.empty() && walls.empty() && trajectory_keypoints.empty(); }
};

/// Load a predefined world preset
/// @param preset The preset to load
/// @return World data for the preset
World load_preset(Preset preset);

/// Generate dense trajectory poses from sparse keypoints
/// @param keypoints Sparse trajectory keypoints (x, y, theta)
/// @param num_poses Target number of poses to generate
/// @return Dense trajectory poses with interpolated positions and computed orientations
std::vector<Eigen::Vector3f> interpolate_trajectory(const std::vector<Eigen::Vector3f>& keypoints, int num_poses = 60);

} // namespace world
