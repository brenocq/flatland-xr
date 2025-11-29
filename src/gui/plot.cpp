// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include "implot.h"
#include <gui/plot.hpp>

void plot_2d_path(const std::string& label, const std::vector<Eigen::Vector2f>& positions, const Color& color, float weight) {
    plot_2d_line(label, positions, color, weight);
}

void plot_2d_line(const std::string& label, const std::vector<Eigen::Vector2f>& positions, const Color& color, float weight) {
    if (positions.empty())
        return;

    ImPlot::SetNextLineStyle(ImVec4(color), weight);
    ImPlot::PlotLine(label.c_str(), &positions[0].x(), &positions[0].y(), positions.size(), ImPlotLineFlags_None, 0, sizeof(Eigen::Vector2f));
}

void plot_2d_scatter(const std::string& label, const std::vector<Eigen::Vector2f>& positions, const Color& color, float size) {
    if (positions.empty())
        return;

    ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, size, ImVec4(color), IMPLOT_AUTO, ImVec4(color));
    ImPlot::PlotScatter(label.c_str(), &positions[0].x(), &positions[0].y(), positions.size(), ImPlotScatterFlags_None, 0, sizeof(Eigen::Vector2f));
}

void plot_2d_camera(const std::string& label, const Eigen::Vector2f& position, float orientation, float fov, float focal_length, const Color& color, float weight) {
    // Calculate the image plane corners
    float half_fov = fov / 2.0f;
    float half_width = focal_length * std::tan(half_fov);

    // Direction vectors
    Eigen::Vector2f forward(std::cos(orientation), std::sin(orientation));
    Eigen::Vector2f right(std::cos(orientation - M_PI / 2.0f), std::sin(orientation - M_PI / 2.0f));

    // Image plane center
    Eigen::Vector2f plane_center = position + forward * focal_length;

    // Image plane corners
    Eigen::Vector2f left_corner = plane_center - right * half_width;
    Eigen::Vector2f right_corner = plane_center + right * half_width;

    // Draw camera frustum (triangle from position to image plane corners)
    std::vector<Eigen::Vector2f> frustum = {position, left_corner, right_corner, position};

    ImPlot::SetNextLineStyle(ImVec4(color), weight);
    ImPlot::PlotLine(label.c_str(), &frustum[0].x(), &frustum[0].y(), frustum.size(), ImPlotLineFlags_None, 0, sizeof(Eigen::Vector2f));
}

void plot_2d_poses(const std::string& label, const std::vector<Eigen::Vector3f>& poses, const Color& color, float weight, float scatter_size) {
    if (poses.empty())
        return;

    // Extract positions from poses
    std::vector<Eigen::Vector2f> positions;
    positions.reserve(poses.size());
    for (const auto& pose : poses) {
        positions.push_back(Eigen::Vector2f(pose.x(), pose.y()));
    }

    plot_2d_line(label, positions, color, weight);
    plot_2d_scatter("##" + label + "_scatter", positions, color, scatter_size);
}
