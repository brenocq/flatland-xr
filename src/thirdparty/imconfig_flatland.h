// Custom ImGui configuration for Flatland-XR
// This file is included by imgui.h via IMGUI_USER_CONFIG define

#pragma once

#include <cmath>

// Enable ImGui's built-in math operators for ImVec2 and ImVec4
#define IMGUI_DEFINE_MATH_OPERATORS

// clang-format off
// Add useful methods to ImVec2
#define IM_VEC2_CLASS_EXTRA                                                              \
    /* Length/magnitude of vector */                                                     \
    float length() const { return std::sqrt(x * x + y * y); }                            \
                                                                                         \
    /* Squared length (faster, avoids sqrt) */                                           \
    float length_squared() const { return x * x + y * y; }                               \
                                                                                         \
    /* Distance to another point */                                                      \
    float distance(const ImVec2& other) const {                                          \
        float dx = x - other.x;                                                          \
        float dy = y - other.y;                                                          \
        return std::sqrt(dx * dx + dy * dy);                                             \
    }                                                                                    \
                                                                                         \
    /* Squared distance (faster, avoids sqrt) */                                         \
    float distance_squared(const ImVec2& other) const {                                  \
        float dx = x - other.x;                                                          \
        float dy = y - other.y;                                                          \
        return dx * dx + dy * dy;                                                        \
    }                                                                                    \
                                                                                         \
    /* Normalize vector (returns zero vector if length is zero) */                       \
    ImVec2 normalized() const {                                                          \
        float len = length();                                                            \
        return len > 0.0f ? ImVec2(x / len, y / len) : ImVec2(0.0f, 0.0f);                \
    }                                                                                    \
                                                                                         \
    /* Dot product */                                                                    \
    float dot(const ImVec2& other) const { return x * other.x + y * other.y; }           \
                                                                                         \
    /* Cross product (returns scalar z-component) */                                     \
    float cross(const ImVec2& other) const { return x * other.y - y * other.x; }         \
                                                                                         \
    /* Perpendicular vector (rotated 90 degrees counter-clockwise) */                    \
    ImVec2 perp() const { return ImVec2(-y, x); }                                        \
                                                                                         \
    /* Linear interpolation */                                                           \
    ImVec2 lerp(const ImVec2& other, float t) const {                                    \
        return ImVec2(x + (other.x - x) * t, y + (other.y - y) * t);                     \
    }

// Note: We intentionally don't add IM_VEC4_CLASS_EXTRA as it can cause
// ambiguity issues with ImRect::Add() in imgui_internal.h
// clang-format on
