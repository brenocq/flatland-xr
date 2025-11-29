// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#pragma once

#include <Eigen/Dense>
#include <gui/color.hpp>
#include <string>
#include <vector>

void plot_2d_path(const std::string& label, const std::vector<Eigen::Vector2f>& positions, const Color& color = Color::Auto(), float weight = -1.0f);
void plot_2d_line(const std::string& label, const std::vector<Eigen::Vector2f>& positions, const Color& color = Color::Auto(), float weight = -1.0f);
void plot_2d_scatter(const std::string& label, const std::vector<Eigen::Vector2f>& positions, const Color& color = Color::Auto(), float size = -1.0f);
void plot_2d_camera(const std::string& label, const Eigen::Vector2f& position, float orientation, float fov, float focal_length,
                    const Color& color = Color::Auto(), float weight = -1.0f);
void plot_2d_poses(const std::string& label, const std::vector<Eigen::Vector3f>& poses, const Color& color = Color::Auto(), float weight = -1.0f,
                   float scatter_size = 1.0f);

class Trajectory2D;
void plot_2d_trajectory(const std::string& label, const Trajectory2D& trajectory, const Color& color = Color::Auto(), float weight = -1.0f,
                        float scatter_size = 1.0f);
