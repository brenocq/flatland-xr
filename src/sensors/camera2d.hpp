// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#pragma once

#include <Eigen/Dense>
#include <optional>
#include <random>
#include <sensors/sensor_data.hpp>
#include <vector>
#include <core/types.hpp>
namespace sensors {

/// 2D Camera model for projecting landmarks using pinhole model
class Camera2D {
  public:
    Camera2D() = default;

    /// Configure camera intrinsics from width and FOV
    /// @param width Image width in pixels
    /// @param fov Field of view in radians
    void set_intrinsics(int width, float fov);

    /// Set image width in pixels (keeps FOV constant by adjusting focal_length)
    void set_width(int width);

    /// Set field of view in radians (adjusts focal_length)
    void set_fov(float fov);

    /// Set measurement noise standard deviation (in pixels)
    void set_noise_std(float noise_std);

    /// Get image width in pixels
    int width() const { return _width; }

    /// Get field of view in radians (computed from focal_length and width)
    float fov() const;

    /// Get focal length in pixels
    float focal_length() const { return _focal_length; }

    /// Get optical center (principal point, cx = width/2)
    float principal_point() const { return _principal_point; }

    /// Get measurement noise standard deviation (in pixels)
    float noise_std() const { return _noise_std; }

    /// Get the intrinsic matrix K (2x3 for 2D: [fx 0 cx; 0 1 0] style, but we use 1D projection)
    /// Returns [fx, cx] for 1D projection: u = fx * (y/x) + cx
    Eigen::Vector2f intrinsics() const { return Eigen::Vector2f(_focal_length, _principal_point); }

    /// Project a single landmark to camera image plane (no noise)
    /// @param pose Camera pose (x, y, orientation)
    /// @param landmark Landmark position in world coordinates
    /// @return Pixel coordinate u if visible, std::nullopt if outside FOV or behind camera
    std::optional<float> project(const Eigen::Vector3f& pose, const Eigen::Vector2f& landmark) const;

    /// Raytrace camera view to detect wall intersections
    /// @param pose Camera pose (x, y, orientation)
    /// @param walls Vector of walls in the environment
    /// @return Pixel color intensities along the image width
    sensors::Image1D raytrace(const Eigen::Vector3f& pose, const std::vector<core::Wall>& walls) const;

    /// Ray march a single ray from origin in direction dir to find wall intersection
    /// @param origin Ray origin in world coordinates
    /// @param dir Normalized ray direction in world coordinates
    /// @param walls Vector of walls in the environment
    /// @return RayHit containing hit information
    core::RayHit march_ray(const Eigen::Vector2f& origin, const Eigen::Vector2f& dir, const std::vector<core::Wall>& walls) const;

    float distance_point_segment(const Eigen::Vector2f& p, const Eigen::Vector2f& a, const Eigen::Vector2f& b) const;
    float signed_distance_to_walls(const Eigen::Vector2f& p, const std::vector<core::Wall>& walls) const;
    core::RayHit find_wall_color_at_hit(const Eigen::Vector2f& p, const std::vector<core::Wall>& walls) const;
    
    /// Project multiple landmarks to camera image plane (no noise)
    /// @param pose Camera pose (x, y, orientation)
    /// @param landmarks Vector of landmark positions
    /// @return Vector of measurements for landmarks within FOV
    std::vector<CameraMeasurement> project_landmarks(const Eigen::Vector3f& pose, const std::vector<Eigen::Vector2f>& landmarks) const;

    /// Generate noisy measurement from ground truth projection
    /// @param pose Camera pose (x, y, orientation)
    /// @param landmark Landmark position in world coordinates
    /// @return Noisy pixel coordinate u if visible, std::nullopt if outside FOV or behind camera
    std::optional<float> measure(const Eigen::Vector3f& pose, const Eigen::Vector2f& landmark);

    /// Generate noisy measurements for multiple landmarks
    /// @param pose Camera pose (x, y, orientation)
    /// @param landmarks Vector of landmark positions
    /// @return Vector of noisy measurements for landmarks within FOV
    std::vector<CameraMeasurement> measure_landmarks(const Eigen::Vector3f& pose, const std::vector<Eigen::Vector2f>& landmarks);

  private:
    int _width = 100;               ///< Image width in pixels
    float _focal_length = 100.0f;   ///< Focal length in pixels (fx)
    float _principal_point = 50.0f; ///< Principal point (cx = width/2)
    float _noise_std = 1.0f;        ///< Measurement noise standard deviation (pixels)

    std::mt19937 _rng{std::random_device{}()}; ///< Random number generator
};

} // namespace sensors
