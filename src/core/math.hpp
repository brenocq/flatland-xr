#pragma once

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

} // namespace core
