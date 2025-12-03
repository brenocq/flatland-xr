// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include "imconfig.h"
#include "implot.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <core/math.hpp>
#include <core/trajectory2d.hpp>
#include <gui/plot.hpp>
#include <memory>
#include <sensors/camera2d.hpp>

namespace gui {

void plot_2d_path(const std::string& label, const std::vector<Eigen::Vector2f>& positions, const Color& color, float weight) {
    plot_2d_line(label, positions, color, weight);
}

void plot_2d_line(const std::string& label, const std::vector<Eigen::Vector2f>& positions, const Color& color, float weight) {
    if (positions.empty())
        return;

    ImPlot::SetNextLineStyle(color, weight);
    ImPlot::PlotLine(label.c_str(), &positions[0].x(), &positions[0].y(), static_cast<int>(positions.size()), ImPlotLineFlags_None, 0,
                     sizeof(Eigen::Vector2f));
}

void plot_2d_scatter(const std::string& label, const std::vector<Eigen::Vector2f>& positions, const Color& color, float size) {
    if (positions.empty())
        return;

    ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, size, color, IMPLOT_AUTO, color);
    ImPlot::PlotScatter(label.c_str(), &positions[0].x(), &positions[0].y(), static_cast<int>(positions.size()), ImPlotScatterFlags_None, 0,
                        sizeof(Eigen::Vector2f));
}

void plot_2d_camera_frustum(const std::string& label, const Eigen::Vector2f& position, float orientation, float fov, float focal_length,
                            const Color& color, float weight) {
    // Calculate the image plane corners
    float half_fov = fov / 2.0f;
    float half_width = focal_length * std::tan(half_fov);

    // Direction vectors
    Eigen::Vector2f forward(std::cos(orientation), std::sin(orientation));
    Eigen::Vector2f right(std::cos(orientation - core::HALF_PI), std::sin(orientation - core::HALF_PI));

    // Image plane center
    Eigen::Vector2f plane_center = position + forward * focal_length;

    // Image plane corners
    Eigen::Vector2f left_corner = plane_center - right * half_width;
    Eigen::Vector2f right_corner = plane_center + right * half_width;

    // Draw camera frustum (triangle from position to image plane corners)
    std::vector<Eigen::Vector2f> frustum = {position, left_corner, right_corner, position};

    ImPlot::SetNextLineStyle(color, weight);
    ImPlot::PlotLine(label.c_str(), &frustum[0].x(), &frustum[0].y(), static_cast<int>(frustum.size()), ImPlotLineFlags_None, 0,
                     sizeof(Eigen::Vector2f));
}

void plot_2d_poses(const std::string& label, const std::vector<Eigen::Vector3f>& poses, const Color& color, float weight, float scatter_size) {
    if (poses.empty())
        return;

    // Extract positions from poses
    std::vector<Eigen::Vector2f> positions;
    positions.reserve(poses.size());
    for (const auto& pose : poses) {
        positions.emplace_back(pose.x(), pose.y());
    }

    plot_2d_line(label, positions, color, weight);
    plot_2d_scatter("##" + label + "_scatter", positions, color, scatter_size);
}

void plot_2d_trajectory(const std::string& label, const core::Trajectory2D& trajectory, const Color& color, float weight, float scatter_size) {
    if (!trajectory.is_valid())
        return;

    float max_t = trajectory.max_t();
    if (max_t <= 0)
        return;

    // Estimate arc length in pixels by sampling coarsely
    const int coarse_samples = 20;
    float total_px_length = 0.0f;
    ImVec2 prev_px = ImPlot::PlotToPixels(ImPlotPoint(trajectory.position(0).x(), trajectory.position(0).y()));

    for (int i = 1; i <= coarse_samples; i++) {
        float t = max_t * static_cast<float>(i) / static_cast<float>(coarse_samples);
        Eigen::Vector2f pos = trajectory.position(t);
        ImVec2 curr_px = ImPlot::PlotToPixels(ImPlotPoint(pos.x(), pos.y()));
        total_px_length += curr_px.distance(prev_px);
        prev_px = curr_px;
    }

    // Use approximately 1 sample per 2 pixels, with reasonable bounds
    int num_samples = static_cast<int>(total_px_length / 2.0f);
    num_samples = std::max(static_cast<int>(trajectory.num_poses()), std::min(num_samples, 2000));

    // Sample the trajectory uniformly
    std::vector<Eigen::Vector2f> sampled_positions;
    sampled_positions.reserve(num_samples);

    for (int i = 0; i < num_samples; i++) {
        float t = max_t * static_cast<float>(i) / static_cast<float>(num_samples - 1);
        sampled_positions.push_back(trajectory.position(t));
    }

    plot_2d_line(label, sampled_positions, color, weight);

    // Plot scatter at integer t values (original poses)
    std::vector<Eigen::Vector2f> pose_positions;
    pose_positions.reserve(trajectory.num_poses());

    for (size_t i = 0; i < trajectory.num_poses(); i++) {
        pose_positions.push_back(trajectory.position(static_cast<float>(i)));
    }

    plot_2d_scatter(label, pose_positions, color, scatter_size);
}

void plot_2d_camera_observations(const std::string& label, const Eigen::Vector2f& position, float orientation,
                                 const std::shared_ptr<sensors::Camera2D> camera, const std::vector<sensors::CameraMeasurement>& observations) {
    if (observations.empty())
        return;

    // Use 1 unit distance for visualization
    constexpr float vis_distance = 1.0f;
    float fov = camera->fov();
    float half_fov = fov / 2.0f;
    float half_width = vis_distance * std::tan(half_fov);

    // Direction vectors
    Eigen::Vector2f forward(std::cos(orientation), std::sin(orientation));
    Eigen::Vector2f right(std::cos(orientation - core::HALF_PI), std::sin(orientation - core::HALF_PI));

    // Image plane center and corners (1 unit away)
    Eigen::Vector2f plane_center = position + forward * vis_distance;
    Eigen::Vector2f left_corner = plane_center - right * half_width;
    Eigen::Vector2f right_corner = plane_center + right * half_width;

    // Draw each observation with its landmark color
    for (const auto& obs : observations) {
        float t = obs.u / static_cast<float>(camera->width());
        Eigen::Vector2f point = left_corner + (right_corner - left_corner) * t;

        Color color = Color::Random(obs.landmark_id);
        ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 4.0f, color, IMPLOT_AUTO, color);
        ImPlot::PlotScatter(label.c_str(), &point.x(), &point.y(), 1, ImPlotScatterFlags_None, 0, sizeof(Eigen::Vector2f));
    }
}

void plot_2d_camera_rays(const std::string& label, const Eigen::Vector2f& position, const std::vector<Eigen::Vector2f>& landmarks,
                         const std::vector<sensors::CameraMeasurement>& observations, float weight) {
    if (observations.empty())
        return;

    for (const auto& obs : observations) {
        const Eigen::Vector2f& landmark = landmarks[obs.landmark_id];
        Color color = Color::Random(obs.landmark_id);

        std::vector<Eigen::Vector2f> ray = {position, landmark};
        ImPlot::SetNextLineStyle(color, weight);
        ImPlot::PlotLine(label.c_str(), &ray[0].x(), &ray[0].y(), static_cast<int>(ray.size()), ImPlotLineFlags_None, 0, sizeof(Eigen::Vector2f));
    }
}

void plot_2d_ray_march(const std::string& label, const Eigen::Vector2f& cam_pos, const std::vector<core::RayHit>& rays, float weight) {
    if (rays.empty())
        return;

    const size_t count = rays.size();
    for (size_t i = 0; i < count; ++i) {
        const Eigen::Vector2f& hit = rays[i].hit_pos;
        if (!std::isfinite(hit.x()) || !std::isfinite(hit.y()))
            continue;
        if ((hit - cam_pos).squaredNorm() < 1e-8f)
            continue;

        std::array<Eigen::Vector2f, 2> segment = {cam_pos, hit};
        const Eigen::Vector3f& color = rays[i].color;
        ImPlot::SetNextLineStyle(ImVec4(color.x(), color.y(), color.z(), 1.0f), weight);
        ImPlot::PlotLine((label + std::to_string(i)).c_str(), &segment[0].x(), &segment[0].y(), 2, ImPlotLineFlags_None, 0,
                         sizeof(Eigen::Vector2f));
    }
}

void plot_pose_highlight(const Eigen::Vector3f& pose) {
    float x = pose.x();
    float y = pose.y();
    ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 4.0f, Color::Transparent(), 1.0f, Color::CatSapphire());
    ImPlot::PlotScatter("##HoverHighlight", &x, &y, 1);
}

} // namespace gui
