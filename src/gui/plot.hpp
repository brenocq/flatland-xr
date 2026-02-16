// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#pragma once

#include <Eigen/Dense>
#include <core/types.hpp>
#include <gui/color.hpp>
#include <memory>
#include <string>
#include <vector>

namespace core {
class Trajectory2D;
}

namespace sensors {
struct CameraMeasurement;
class Camera2D;
} // namespace sensors

namespace gui {

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
void plot_2d_camera_observations(const std::string& label, const Eigen::Vector2f& position, float orientation,
                                 const std::shared_ptr<sensors::Camera2D> camera, const std::vector<sensors::CameraMeasurement>& observations);

/// Plot rays from camera center to observed landmarks (colored by landmark ID)
void plot_2d_camera_rays(const std::string& label, const Eigen::Vector2f& position, const std::vector<Eigen::Vector2f>& landmarks,
                         const std::vector<sensors::CameraMeasurement>& observations, float weight = -1.0f);

/// Plot ray-marched hits from the camera center to impact points colored by wall color
void plot_2d_ray_march(const std::string& label, const Eigen::Vector2f& cam_pos, const std::vector<core::RayHit>& rays, float weight = 1.0f);

/// Plot an arrow from position to position+vector
/// @param label Label for the plot
/// @param position Starting position of the arrow
/// @param vector Direction and magnitude of the arrow
/// @param color Arrow color
/// @param weight Line weight for the arrow shaft
/// @param head_size Size of the arrowhead (default auto-sized based on vector length)
void plot_2d_arrow(const std::string& label, const Eigen::Vector2f& position, const Eigen::Vector2f& vector, const Color& color = Color::Auto(),
                   float weight = -1.0f, float head_size = -1.0f);

/// Plot a function y = f(x) by sampling it at screen resolution
/// @param label Label for the plot
/// @param func Callable that takes a float (x) and returns a float (y)
/// @param color Line color
/// @param weight Line weight
template <typename Func> void plot_2d_func(const std::string& label, Func func, const Color& color = Color::Auto(), float weight = -1.0f);

/// Draw a highlight circle at the specified pose position (call inside active ImPlot context)
void plot_pose_highlight(const Eigen::Vector3f& pose);

/// Plot multiple lines from a vector of N-dimensional vectors
/// @tparam Size Dimension of the vectors (e.g., 2 for Vec2, 3 for Vec3)
/// @param label Base label for the lines (e.g., "Acc" -> "Acc X", "Acc Y")
/// @param time_axis X-axis values (usually time indices)
/// @param data Vector of N-dimensional vectors
/// @param colors Colors for each dimension (defaults to DefaultPalette)
/// @param line_width Line width (default -1.0f = auto)
template <int Size> void plot_vector(const std::string& label, const std::vector<float>& time_axis,
                                     const std::vector<Eigen::Vector<float, Size>>& data, const std::vector<Color>& colors = Color::DefaultPalette(),
                                     float line_width = -1.0f);

/// Plot covariance bands (mean ± 3*std) for N-dimensional vectors
/// @tparam Size Dimension of the vectors (e.g., 2 for Vec2, 3 for Vec3)
/// @param label Base label (e.g., "Est" -> "Est X Cov", "Est Y Cov")
/// @param time_axis X-axis values (usually time indices)
/// @param mean Vector of mean values
/// @param std_dev Vector of standard deviations
/// @param colors Colors for each dimension (defaults to DefaultPalette with 0.1 alpha)
template <int Size> void plot_covariance(const std::string& label, const std::vector<float>& time_axis,
                                         const std::vector<Eigen::Vector<float, Size>>& mean, const std::vector<Eigen::Vector<float, Size>>& std_dev,
                                         const std::vector<Color>& colors = Color::DefaultPalette());

} // namespace gui

// Template implementation
#include <gui/plot_impl.hpp>
