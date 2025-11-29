// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#pragma once

#include "imgui.h"
#include <Eigen/Dense>

class Color : public Eigen::Vector4f {
  public:
    Color(float r, float g, float b, float a = 1.0f) : Eigen::Vector4f(r, g, b, a) {}

    explicit operator ImVec4() const { return ImVec4(r(), g(), b(), a()); }
    explicit Color(const ImVec4& c) : Eigen::Vector4f(c.x, c.y, c.z, c.w) {}

    float r() const { return x(); }
    float g() const { return y(); }
    float b() const { return z(); }
    float a() const { return w(); }
    float& r() { return x(); }
    float& g() { return y(); }
    float& b() { return z(); }
    float& a() { return w(); }

    static Color Auto() { return Color(0.0f, 0.0f, 0.0f, -1.0f); }
    static Color Red() { return Color(1.0f, 0.0f, 0.0f); }
    static Color Green() { return Color(0.0f, 1.0f, 0.0f); }
    static Color Blue() { return Color(0.0f, 0.0f, 1.0f); }
};
