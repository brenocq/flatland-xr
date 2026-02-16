#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace core {

// Common mathematical constants as inline constexpr for zero-overhead usage
// Float-only since this codebase uses float throughout

inline constexpr float PI = 3.14159265358979323846F;
inline constexpr float HALF_PI = 1.57079632679489661923F;
inline constexpr float QUARTER_PI = 0.78539816339744830962F;
inline constexpr float TWO_PI = 6.28318530717958647692F;
inline constexpr float DEG_TO_RAD = 0.01745329251994329577F;
inline constexpr float RAD_TO_DEG = 57.29577951308232087680F;

// Convenience conversion functions
constexpr float deg_to_rad(float degrees) { return degrees * DEG_TO_RAD; }

constexpr float rad_to_deg(float radians) { return radians * RAD_TO_DEG; }

// Euclidean distance helper (avoids sqrtf/powf which aren't standard)
inline float distance(float dx, float dy) { return static_cast<float>(std::sqrt(static_cast<double>(dx * dx + dy * dy))); }

using Vec1f = Eigen::Vector<float, 1>;
using Vec2f = Eigen::Vector<float, 2>;
using Vec3f = Eigen::Vector<float, 3>;
using Vec4f = Eigen::Vector<float, 4>;
using Vec5f = Eigen::Vector<float, 5>;
using Vec6f = Eigen::Vector<float, 6>;
using Vec7f = Eigen::Vector<float, 7>;
using Vec8f = Eigen::Vector<float, 8>;
using Vec9f = Eigen::Vector<float, 9>;
using Vec10f = Eigen::Vector<float, 10>;

using Vec1d = Eigen::Vector<double, 1>;
using Vec2d = Eigen::Vector<double, 2>;
using Vec3d = Eigen::Vector<double, 3>;
using Vec4d = Eigen::Vector<double, 4>;
using Vec5d = Eigen::Vector<double, 5>;
using Vec6d = Eigen::Vector<double, 6>;
using Vec7d = Eigen::Vector<double, 7>;
using Vec8d = Eigen::Vector<double, 8>;
using Vec9d = Eigen::Vector<double, 9>;
using Vec10d = Eigen::Vector<double, 10>;

using Mat2f = Eigen::Matrix<float, 2, 2>;
using Mat3f = Eigen::Matrix<float, 3, 3>;
using Mat4f = Eigen::Matrix<float, 4, 4>;
using Mat5f = Eigen::Matrix<float, 5, 5>;
using Mat6f = Eigen::Matrix<float, 6, 6>;
using Mat7f = Eigen::Matrix<float, 7, 7>;
using Mat8f = Eigen::Matrix<float, 8, 8>;
using Mat9f = Eigen::Matrix<float, 9, 9>;
using Mat10f = Eigen::Matrix<float, 10, 10>;

using Mat2d = Eigen::Matrix<double, 2, 2>;
using Mat3d = Eigen::Matrix<double, 3, 3>;
using Mat4d = Eigen::Matrix<double, 4, 4>;
using Mat5d = Eigen::Matrix<double, 5, 5>;
using Mat6d = Eigen::Matrix<double, 6, 6>;
using Mat7d = Eigen::Matrix<double, 7, 7>;
using Mat8d = Eigen::Matrix<double, 8, 8>;
using Mat9d = Eigen::Matrix<double, 9, 9>;
using Mat10d = Eigen::Matrix<double, 10, 10>;

} // namespace core
