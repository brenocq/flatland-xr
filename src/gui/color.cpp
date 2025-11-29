// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include <array>
#include <gui/color.hpp>

namespace {
// 64 distinct, visually pleasing colors shuffled for maximum contrast between adjacent indices
constexpr std::array<std::array<float, 3>, 64> palette = {{
    {0.894f, 0.102f, 0.110f}, // Crimson
    {0.216f, 0.494f, 0.722f}, // Steel Blue
    {0.302f, 0.686f, 0.290f}, // Forest Green
    {0.596f, 0.306f, 0.639f}, // Rebecca Purple
    {1.000f, 0.498f, 0.000f}, // Orange
    {0.251f, 0.878f, 0.816f}, // Turquoise
    {0.969f, 0.506f, 0.749f}, // Hot Pink
    {0.498f, 0.498f, 0.000f}, // Olive
    {0.118f, 0.565f, 1.000f}, // Dodger Blue
    {0.804f, 0.361f, 0.361f}, // Indian Red
    {0.400f, 0.804f, 0.667f}, // Medium Aquamarine
    {0.729f, 0.333f, 0.827f}, // Medium Orchid
    {0.855f, 0.647f, 0.125f}, // Goldenrod
    {0.000f, 0.502f, 0.000f}, // Green
    {0.255f, 0.412f, 0.882f}, // Royal Blue
    {1.000f, 0.388f, 0.278f}, // Tomato
    {0.565f, 0.933f, 0.565f}, // Light Green
    {0.502f, 0.000f, 0.502f}, // Purple
    {0.957f, 0.643f, 0.376f}, // Sandy Brown
    {0.000f, 0.808f, 0.820f}, // Dark Cyan
    {0.867f, 0.627f, 0.867f}, // Plum
    {0.604f, 0.804f, 0.196f}, // Yellow Green
    {0.282f, 0.239f, 0.545f}, // Dark Slate Blue
    {0.941f, 0.502f, 0.502f}, // Light Coral
    {0.180f, 0.545f, 0.341f}, // Sea Green
    {0.576f, 0.439f, 0.859f}, // Medium Purple
    {1.000f, 0.847f, 0.094f}, // Golden Yellow
    {0.098f, 0.098f, 0.439f}, // Midnight Blue
    {0.780f, 0.082f, 0.522f}, // Medium Violet Red
    {0.686f, 0.933f, 0.933f}, // Pale Turquoise
    {0.545f, 0.271f, 0.075f}, // Saddle Brown
    {0.392f, 0.584f, 0.929f}, // Cornflower Blue
    {0.698f, 0.133f, 0.133f}, // Firebrick
    {0.596f, 0.984f, 0.596f}, // Pale Green
    {0.855f, 0.439f, 0.839f}, // Orchid
    {0.824f, 0.412f, 0.118f}, // Chocolate
    {0.529f, 0.808f, 0.922f}, // Sky Blue
    {0.133f, 0.545f, 0.133f}, // Forest
    {0.933f, 0.510f, 0.933f}, // Violet
    {0.741f, 0.718f, 0.420f}, // Dark Khaki
    {0.000f, 0.749f, 1.000f}, // Deep Sky Blue
    {0.863f, 0.078f, 0.235f}, // Crimson Rose
    {0.275f, 0.510f, 0.706f}, // Steel Blue 2
    {1.000f, 0.714f, 0.757f}, // Light Pink
    {0.000f, 0.392f, 0.000f}, // Dark Green
    {0.416f, 0.353f, 0.804f}, // Slate Blue
    {0.914f, 0.588f, 0.478f}, // Dark Salmon
    {0.678f, 0.847f, 0.902f}, // Light Blue
    {0.859f, 0.439f, 0.576f}, // Pale Violet Red
    {0.933f, 0.910f, 0.667f}, // Khaki
    {0.000f, 0.000f, 0.804f}, // Medium Blue
    {0.737f, 0.561f, 0.561f}, // Rosy Brown
    {1.000f, 0.627f, 0.478f}, // Light Salmon
    {0.690f, 0.769f, 0.871f}, // Light Steel Blue
    {1.000f, 0.412f, 0.706f}, // Hot Pink Light
    {0.804f, 0.522f, 0.247f}, // Peru
    {0.502f, 0.000f, 0.000f}, // Maroon
    {1.000f, 0.922f, 0.612f}, // Pale Goldenrod
    {0.600f, 0.600f, 0.600f}, // Gray
    {0.871f, 0.722f, 0.529f}, // Burlywood
    {0.412f, 0.412f, 0.412f}, // Dim Gray
    {0.961f, 0.871f, 0.702f}, // Wheat
    {0.663f, 0.663f, 0.663f}, // Dark Gray
    {0.439f, 0.502f, 0.565f}, // Slate Gray
}};
} // namespace

Color Color::Random(size_t index) {
    const auto& c = palette[index % palette.size()];
    return Color(c[0], c[1], c[2]);
}
