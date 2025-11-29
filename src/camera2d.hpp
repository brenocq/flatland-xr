// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#pragma once

#include <Eigen/Dense>
#include <optional>
#include <vector>

/// Observation of a landmark from a camera
struct LandmarkObservation {
    float u;            ///< Pixel coordinate on the 1D image plane (0 = left edge, width = right edge)
    size_t landmark_id; ///< ID of the observed landmark
};

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

    /// Get image width in pixels
    int width() const { return _width; }

    /// Get field of view in radians (computed from focal_length and width)
    float fov() const;

    /// Get focal length in pixels
    float focal_length() const { return _focal_length; }

    /// Get optical center (principal point, cx = width/2)
    float principal_point() const { return _principal_point; }

    /// Get the intrinsic matrix K (2x3 for 2D: [fx 0 cx; 0 1 0] style, but we use 1D projection)
    /// Returns [fx, cx] for 1D projection: u = fx * (y/x) + cx
    Eigen::Vector2f intrinsics() const { return Eigen::Vector2f(_focal_length, _principal_point); }

    /// Project a single landmark to camera image plane using homogeneous coordinates
    /// @param pose Camera pose (x, y, orientation)
    /// @param landmark Landmark position in world coordinates
    /// @return Pixel coordinate u if visible, std::nullopt if outside FOV or behind camera
    std::optional<float> project(const Eigen::Vector3f& pose, const Eigen::Vector2f& landmark) const;

    /// Project multiple landmarks to camera image plane
    /// @param pose Camera pose (x, y, orientation)
    /// @param landmarks Vector of landmark positions
    /// @return Vector of observations for landmarks within FOV
    std::vector<LandmarkObservation> project_landmarks(const Eigen::Vector3f& pose, const std::vector<Eigen::Vector2f>& landmarks) const;

  private:
    int _width = 100;               ///< Image width in pixels
    float _focal_length = 100.0f;   ///< Focal length in pixels (fx)
    float _principal_point = 50.0f; ///< Principal point (cx = width/2)
};
