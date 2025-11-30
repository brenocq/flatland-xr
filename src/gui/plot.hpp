// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#pragma once

#include <Eigen/Dense>
#include <gui/color.hpp>
#include <string>
#include <vector>

namespace core {
class Trajectory2D;
}

namespace sensors {
struct LandmarkObservation;
class Camera2D;
} // namespace sensors

void plot_2d_path(const std::string& label, const std::vector<Eigen::Vector2f>& positions, const Color& color = Color::Auto(), float weight = -1.0f);
void plot_2d_line(const std::string& label, const std::vector<Eigen::Vector2f>& positions, const Color& color = Color::Auto(), float weight = -1.0f);
void plot_2d_scatter(const std::string& label, const std::vector<Eigen::Vector2f>& positions, const Color& color = Color::Auto(), float size = -1.0f);
void plot_2d_camera_frustum(const std::string& label, const Eigen::Vector2f& position, float orientation, float fov, float focal_length,
                            const Color& color = Color::Auto(), float weight = -1.0f);
void plot_2d_poses(const std::string& label, const std::vector<Eigen::Vector3f>& poses, const Color& color = Color::Auto(), float weight = -1.0f,
                   float scatter_size = 1.0f);

void plot_2d_trajectory(const std::string& label, const core::Trajectory2D& trajectory, const Color& color = Color::Auto(), float weight = -1.0f,
                        float scatter_size = 1.0f);

/// Plot landmark observations as points on the camera image plane (colored by landmark ID)
void plot_2d_camera_observations(const std::string& label, const Eigen::Vector2f& position, float orientation, const sensors::Camera2D& camera,
                                 const std::vector<sensors::LandmarkObservation>& observations);

/// Plot rays from camera center to observed landmarks (colored by landmark ID)
void plot_2d_camera_rays(const std::string& label, const Eigen::Vector2f& position, const std::vector<Eigen::Vector2f>& landmarks,
                         const std::vector<sensors::LandmarkObservation>& observations, float weight = -1.0f);
