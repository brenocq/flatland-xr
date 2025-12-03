// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#pragma once

#include <Eigen/Dense>
#include <core/types.hpp>
#include <vector>

namespace sensors {

/// IMU measurement containing accelerometer and gyroscope readings
struct IMUMeasurement {
    Eigen::Vector2f acc = Eigen::Vector2f::Zero(); ///< Accelerometer reading (x, y) in body frame
    float gyr = 0.0f;                              ///< Gyroscope reading in rad/s

    IMUMeasurement() = default;
    IMUMeasurement(const Eigen::Vector2f& a, float g) : acc(a), gyr(g) {}
    IMUMeasurement(float ax, float ay, float g) : acc(ax, ay), gyr(g) {}
};

/// Camera measurement of a landmark
struct CameraMeasurement {
    float u = 0.0f;         ///< Pixel coordinate on the 1D image plane (0 = left edge, width = right edge)
    size_t landmark_id = 0; ///< ID of the observed landmark

    CameraMeasurement() = default;
    CameraMeasurement(float u_, size_t id) : u(u_), landmark_id(id) {}
};

struct Image1D {
    std::vector<core::RayHit> rays; ///< Ray hit information per pixel

    Image1D() = default;
    explicit Image1D(size_t width) : rays(width) {}
    explicit Image1D(const std::vector<core::RayHit>& r) : rays(r) {}

    bool empty() const { return rays.empty(); }
    size_t size() const { return rays.size(); }
};

/// Camera frame containing all measurements at a single time step
struct CameraFrame {
    std::vector<CameraMeasurement> observations;
    Image1D image;

    CameraFrame() = default;
    CameraFrame(const std::vector<CameraMeasurement>& obs) : observations(obs) {}

    bool empty() const { return observations.empty(); }
    size_t size() const { return observations.size(); }
};

} // namespace sensors
