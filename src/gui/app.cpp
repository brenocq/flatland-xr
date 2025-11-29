// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include "imgui.h"
#include "implot.h"

#include <cmath>
#include <gui/app.hpp>
#include <gui/plot.hpp>

App::App() {}

void App::startup() {
    _camera.set_intrinsics(100, 60.0f * M_PI / 180.0f);
    _camera.set_noise_std(1.0f);
    _imu.set_acc_bias(Eigen::Vector2f(0.0f, 0.0f));
    _imu.set_gyr_bias(0.0f);
    _imu.set_acc_noise_std(Eigen::Vector2f(0.01f, 0.01f));
    _imu.set_gyr_noise_std(0.001f);
    load_world_preset(WorldPreset::ASquaresHouse); // Load default preset
}

void App::shutdown() {}

void App::load_world_preset(WorldPreset preset) {
    _current_preset = preset;
    _gt_pose_raw.clear();
    _gt_trajectory = Trajectory2D();
    _landmarks.clear();
    _walls.clear();
    _wall_raw_points.clear();

    if (preset == WorldPreset::Custom) {
        return;
    }

    std::vector<Eigen::Vector3f> poses;

    // Helper to create a regular polygon (habitant) as walls with landmarks at vertices
    // Returns the center for reference
    auto create_polygon = [&](Eigen::Vector2f center, float size, int sides, float rotation = 0.0f, float landmark_offset = 0.15f) {
        std::vector<Eigen::Vector2f> vertices;
        for (int i = 0; i < sides; i++) {
            float angle = rotation + i * 2.0f * M_PI / sides;
            vertices.push_back(center + Eigen::Vector2f(size * std::cos(angle), size * std::sin(angle)));
        }
        // Create walls
        for (int i = 0; i < sides; i++) {
            Wall wall;
            wall.points.push_back(vertices[i]);
            wall.points.push_back(vertices[(i + 1) % sides]);
            _walls.push_back(wall);
        }
        // Create landmarks slightly outside each vertex
        for (int i = 0; i < sides; i++) {
            Eigen::Vector2f dir = (vertices[i] - center).normalized();
            _landmarks.push_back(vertices[i] + dir * landmark_offset);
        }
    };

    // Helper to create a line (wife/daughter in Flatland) as a wall with landmarks at ends
    auto create_line = [&](Eigen::Vector2f center, float length, float angle, float landmark_offset = 0.15f) {
        Eigen::Vector2f dir(std::cos(angle), std::sin(angle));
        Eigen::Vector2f p1 = center - dir * (length / 2.0f);
        Eigen::Vector2f p2 = center + dir * (length / 2.0f);
        Wall wall;
        wall.points.push_back(p1);
        wall.points.push_back(p2);
        _walls.push_back(wall);
        // Landmarks at the endpoints, slightly offset outward
        _landmarks.push_back(p1 - dir * landmark_offset);
        _landmarks.push_back(p2 + dir * landmark_offset);
    };

    // Helper to create a circle (priest/ruler) approximated as many-sided polygon
    auto create_circle = [&](Eigen::Vector2f center, float radius, int segments = 16, float landmark_offset = 0.15f) {
        std::vector<Eigen::Vector2f> vertices;
        for (int i = 0; i < segments; i++) {
            float angle = i * 2.0f * M_PI / segments;
            vertices.push_back(center + Eigen::Vector2f(radius * std::cos(angle), radius * std::sin(angle)));
        }
        // Create walls
        for (int i = 0; i < segments; i++) {
            Wall wall;
            wall.points.push_back(vertices[i]);
            wall.points.push_back(vertices[(i + 1) % segments]);
            _walls.push_back(wall);
        }
        // Add landmarks at cardinal points only (to avoid too many)
        for (int i = 0; i < 4; i++) {
            float angle = i * M_PI / 2.0f;
            Eigen::Vector2f dir(std::cos(angle), std::sin(angle));
            _landmarks.push_back(center + dir * (radius + landmark_offset));
        }
    };

    if (preset == WorldPreset::ASquaresHouse) {
        // A Square's pentagonal house - higher class Flatlanders have more sides
        // Pentagon house walls
        const float house_radius = 10.0f;
        std::vector<Eigen::Vector2f> pentagon;
        for (int i = 0; i < 5; i++) {
            float angle = M_PI / 2.0f + i * 2.0f * M_PI / 5.0f; // Start from top
            pentagon.push_back(Eigen::Vector2f(house_radius * std::cos(angle), house_radius * std::sin(angle)));
        }
        for (int i = 0; i < 5; i++) {
            Wall wall;
            wall.points.push_back(pentagon[i]);
            wall.points.push_back(pentagon[(i + 1) % 5]);
            _walls.push_back(wall);
        }

        // A Square (narrator) - a square shape in the center-left area
        create_polygon(Eigen::Vector2f(-3.0f, 1.0f), 0.8f, 4, M_PI / 4.0f);

        // Wife (line) near southern door - lines are dangerous in Flatland!
        create_line(Eigen::Vector2f(0.0f, -6.0f), 1.5f, M_PI / 2.0f);

        // Son 1 - an equilateral triangle (lower class)
        create_polygon(Eigen::Vector2f(-4.0f, 4.0f), 0.7f, 3, -M_PI / 2.0f);

        // Son 2 - a square (same class as father)
        create_polygon(Eigen::Vector2f(4.0f, 3.0f), 0.6f, 4, M_PI / 4.0f);

        // Grandson - isosceles triangle (soldier class)
        create_polygon(Eigen::Vector2f(2.0f, -3.0f), 0.5f, 3, M_PI / 2.0f);

        // Furniture corners as small triangles
        create_polygon(Eigen::Vector2f(-5.0f, -2.0f), 0.4f, 3, 0.0f);
        create_polygon(Eigen::Vector2f(5.0f, -2.0f), 0.4f, 3, M_PI);

        // Trajectory: A Square moving around his house visiting family
        poses = {
            {-6.0f, -4.0f, M_PI / 4.0f}, {-5.0f, -1.0f, M_PI / 2.0f},        {-5.0f, 3.0f, M_PI / 3.0f},  {-3.0f, 5.0f, 0.0f},
            {2.0f, 5.0f, -M_PI / 4.0f},  {5.0f, 2.0f, -M_PI / 2.0f},         {5.0f, -2.0f, -M_PI / 2.0f}, {3.0f, -5.0f, M_PI},
            {-1.0f, -5.0f, M_PI},        {-4.0f, -3.0f, 2.0f * M_PI / 3.0f},
        };

    } else if (preset == WorldPreset::VisitFromSphere) {
        // The Sphere visits A Square - the climactic scene
        // Simple rectangular room
        float w = 12.0f, h = 10.0f;
        _walls.push_back({{Eigen::Vector2f(-w, -h), Eigen::Vector2f(w, -h)}}); // Bottom
        _walls.push_back({{Eigen::Vector2f(w, -h), Eigen::Vector2f(w, h)}});   // Right
        _walls.push_back({{Eigen::Vector2f(w, h), Eigen::Vector2f(-w, h)}});   // Top
        _walls.push_back({{Eigen::Vector2f(-w, h), Eigen::Vector2f(-w, -h)}}); // Left

        // The Sphere - appears as a circle that grows and shrinks
        // Show it at its maximum cross-section
        create_circle(Eigen::Vector2f(5.0f, 0.0f), 2.5f, 20);

        // A Square (the narrator) observing in amazement
        create_polygon(Eigen::Vector2f(-6.0f, 0.0f), 0.8f, 4, M_PI / 4.0f);

        // Wife hiding in corner (lines are shy/dangerous)
        create_line(Eigen::Vector2f(-9.0f, -7.0f), 1.2f, M_PI / 4.0f);

        // Furniture - small triangular tables
        create_polygon(Eigen::Vector2f(-8.0f, 6.0f), 0.5f, 3, 0.0f);
        create_polygon(Eigen::Vector2f(8.0f, 6.0f), 0.5f, 3, M_PI);
        create_polygon(Eigen::Vector2f(8.0f, -6.0f), 0.5f, 3, M_PI);

        // Trajectory: A Square approaching and circling the Sphere in wonder
        poses = {
            {-10.0f, 0.0f, 0.0f},        {-7.0f, 0.0f, 0.0f},
            {-4.0f, 2.0f, M_PI / 6.0f},  {-1.0f, 5.0f, M_PI / 4.0f},
            {3.0f, 6.0f, 0.0f},          {7.0f, 4.0f, -M_PI / 4.0f},
            {9.0f, 0.0f, -M_PI / 2.0f},  {7.0f, -4.0f, -3.0f * M_PI / 4.0f},
            {3.0f, -6.0f, M_PI},         {-1.0f, -5.0f, 3.0f * M_PI / 4.0f},
            {-4.0f, -2.0f, M_PI / 2.0f}, {-6.0f, 1.0f, M_PI / 4.0f},
        };

    } else if (preset == WorldPreset::HallOfCouncil) {
        // The grand circular hall where the ruling Circles meet
        // Circular hall with many-sided approximation (dodecagon)
        const float outer_radius = 18.0f;
        const int sides = 12;
        std::vector<Eigen::Vector2f> hall;
        for (int i = 0; i < sides; i++) {
            float angle = i * 2.0f * M_PI / sides;
            hall.push_back(Eigen::Vector2f(outer_radius * std::cos(angle), outer_radius * std::sin(angle)));
        }
        for (int i = 0; i < sides; i++) {
            Wall wall;
            wall.points.push_back(hall[i]);
            wall.points.push_back(hall[(i + 1) % sides]);
            _walls.push_back(wall);
        }

        // Inner podium (hexagon)
        const float inner_radius = 5.0f;
        std::vector<Eigen::Vector2f> podium;
        for (int i = 0; i < 6; i++) {
            float angle = M_PI / 6.0f + i * 2.0f * M_PI / 6.0f;
            podium.push_back(Eigen::Vector2f(inner_radius * std::cos(angle), inner_radius * std::sin(angle)));
        }
        for (int i = 0; i < 6; i++) {
            Wall wall;
            wall.points.push_back(podium[i]);
            wall.points.push_back(podium[(i + 1) % 6]);
            _walls.push_back(wall);
        }

        // The ruling Circles (high priests) - positioned around the hall
        create_circle(Eigen::Vector2f(11.0f, 0.0f), 1.2f, 12);
        create_circle(Eigen::Vector2f(-11.0f, 0.0f), 1.2f, 12);
        create_circle(Eigen::Vector2f(0.0f, 11.0f), 1.2f, 12);
        create_circle(Eigen::Vector2f(0.0f, -11.0f), 1.2f, 12);
        create_circle(Eigen::Vector2f(8.0f, 8.0f), 1.0f, 12);
        create_circle(Eigen::Vector2f(-8.0f, 8.0f), 1.0f, 12);
        create_circle(Eigen::Vector2f(-8.0f, -8.0f), 1.0f, 12);
        create_circle(Eigen::Vector2f(8.0f, -8.0f), 1.0f, 12);

        // Isosceles guards (triangles) at entrances
        create_polygon(Eigen::Vector2f(15.0f, 0.0f), 0.8f, 3, M_PI);
        create_polygon(Eigen::Vector2f(-15.0f, 0.0f), 0.8f, 3, 0.0f);

        // Central speaker - a high-ranking polygon (octagon)
        create_polygon(Eigen::Vector2f(0.0f, 0.0f), 1.0f, 8, M_PI / 8.0f);

        // A Square observing from the audience
        create_polygon(Eigen::Vector2f(13.0f, 5.0f), 0.6f, 4, M_PI / 4.0f);

        // Trajectory: A Square entering and observing the council
        poses = {
            {16.0f, 2.0f, M_PI},
            {14.0f, 3.0f, 2.0f * M_PI / 3.0f},
            {10.0f, 6.0f, 2.0f * M_PI / 3.0f},
            {6.0f, 8.0f, M_PI / 2.0f},
            {0.0f, 9.0f, M_PI / 2.0f},
            {-6.0f, 8.0f, 2.0f * M_PI / 3.0f},
            {-10.0f, 5.0f, 5.0f * M_PI / 6.0f},
            {-12.0f, 0.0f, M_PI},
            {-10.0f, -5.0f, -5.0f * M_PI / 6.0f},
            {-6.0f, -8.0f, -2.0f * M_PI / 3.0f},
            {0.0f, -9.0f, -M_PI / 2.0f},
            {6.0f, -8.0f, -M_PI / 3.0f},
            {10.0f, -5.0f, -M_PI / 6.0f},
            {13.0f, 0.0f, 0.0f},
        };
    }

    // Build trajectory from preset poses by interpolating between keypoints
    if (poses.size() >= 2) {
        const int total_poses = 60; // Target number of poses
        std::vector<Eigen::Vector3f> dense_poses;
        dense_poses.reserve(total_poses);

        // Calculate total path length for uniform distribution
        float total_length = 0.0f;
        std::vector<float> segment_lengths;
        for (size_t i = 0; i + 1 < poses.size(); i++) {
            float dx = poses[i + 1].x() - poses[i].x();
            float dy = poses[i + 1].y() - poses[i].y();
            float len = std::sqrt(dx * dx + dy * dy);
            segment_lengths.push_back(len);
            total_length += len;
        }

        // Generate poses uniformly along the path
        float step = total_length / (total_poses - 1);
        float accumulated = 0.0f;
        size_t seg_idx = 0;
        float seg_progress = 0.0f;

        for (int i = 0; i < total_poses; i++) {
            float target_dist = i * step;

            // Find which segment we're on
            while (seg_idx < segment_lengths.size() - 1 && accumulated + segment_lengths[seg_idx] < target_dist) {
                accumulated += segment_lengths[seg_idx];
                seg_idx++;
            }

            // Interpolate within segment
            float seg_len = segment_lengths[seg_idx];
            float t = (seg_len > 0.0f) ? (target_dist - accumulated) / seg_len : 0.0f;
            t = std::clamp(t, 0.0f, 1.0f);

            float x = poses[seg_idx].x() + t * (poses[seg_idx + 1].x() - poses[seg_idx].x());
            float y = poses[seg_idx].y() + t * (poses[seg_idx + 1].y() - poses[seg_idx].y());

            // Compute orientation from movement direction
            float orientation = 0.0f;
            if (!dense_poses.empty()) {
                float dx = x - dense_poses.back().x();
                float dy = y - dense_poses.back().y();
                if (dx != 0 || dy != 0) {
                    orientation = std::atan2(dy, dx);
                } else {
                    orientation = dense_poses.back().z();
                }
            } else if (poses.size() > 1) {
                float dx = poses[1].x() - poses[0].x();
                float dy = poses[1].y() - poses[0].y();
                orientation = std::atan2(dy, dx);
            }

            dense_poses.push_back(Eigen::Vector3f(x, y, orientation));
        }

        _gt_trajectory.build(dense_poses);
    }
}

void App::update() {
    // Docking
    ImGuiID dockId = ImGui::GetID("##DockSpace");
    ImGui::DockSpaceOverViewport(dockId, ImGui::GetMainViewport());
    ImGui::SetNextWindowDockID(dockId, ImGuiCond_Appearing);
    if (ImGui::Begin("Flatland XR")) {
        render();
    }
    ImGui::End();

    // ImGui::ShowDemoWindow();
    // ImPlot::ShowDemoWindow();
}

void App::render() {
    if (render_config() || _first_render) {
        _first_render = false;
        simulate();
        estimate();
    }
    render_world_editor();
    render_measurements();
    render_perception_output();
    render_error_metrics();
}

void App::simulate() {
    // Build trajectory from raw poses if available
    if (!_gt_pose_raw.empty()) {
        build_trajectory_from_raw_poses();
    }
    _est_pos.clear();
}

void App::build_trajectory_from_raw_poses() {
    if (_gt_pose_raw.size() < 4) {
        _gt_trajectory = Trajectory2D();
        return;
    }

    // Smooth poses using a simple moving average filter
    std::vector<Eigen::Vector3f> smoothed_poses;
    smoothed_poses.reserve(_gt_pose_raw.size());

    const int window = 3; // Smoothing window half-size
    for (size_t i = 0; i < _gt_pose_raw.size(); i++) {
        float sum_x = 0, sum_y = 0;
        int count = 0;

        for (int j = -window; j <= window; j++) {
            int idx = static_cast<int>(i) + j;
            if (idx >= 0 && idx < static_cast<int>(_gt_pose_raw.size())) {
                sum_x += _gt_pose_raw[idx].x();
                sum_y += _gt_pose_raw[idx].y();
                count++;
            }
        }

        float avg_x = sum_x / count;
        float avg_y = sum_y / count;

        // Compute orientation from smoothed positions
        float orientation = 0.0f;
        if (i > 0) {
            float dx = avg_x - smoothed_poses.back().x();
            float dy = avg_y - smoothed_poses.back().y();
            if (dx != 0 || dy != 0) {
                orientation = std::atan2(dy, dx);
            } else {
                orientation = smoothed_poses.back().z();
            }
        }

        smoothed_poses.push_back(Eigen::Vector3f(avg_x, avg_y, orientation));
    }

    // Subsample to reduce number of poses (keep roughly every Nth pose)
    std::vector<Eigen::Vector3f> subsampled;
    const size_t target_poses = std::min(smoothed_poses.size(), static_cast<size_t>(100));
    size_t step = std::max(static_cast<size_t>(1), smoothed_poses.size() / target_poses);

    for (size_t i = 0; i < smoothed_poses.size(); i += step) {
        subsampled.push_back(smoothed_poses[i]);
    }
    // Always include the last pose
    if (subsampled.back().x() != smoothed_poses.back().x() || subsampled.back().y() != smoothed_poses.back().y()) {
        subsampled.push_back(smoothed_poses.back());
    }

    // Recompute orientations after subsampling
    for (size_t i = 1; i < subsampled.size(); i++) {
        float dx = subsampled[i].x() - subsampled[i - 1].x();
        float dy = subsampled[i].y() - subsampled[i - 1].y();
        if (dx != 0 || dy != 0) {
            subsampled[i].z() = std::atan2(dy, dx);
        }
    }
    if (subsampled.size() > 1) {
        subsampled[0].z() = subsampled[1].z();
    }

    _gt_trajectory.build(subsampled);
}

void App::build_wall_from_raw_points() {
    if (_wall_raw_points.size() < 2) {
        return;
    }

    // Smooth wall points using moving average
    std::vector<Eigen::Vector2f> smoothed;
    smoothed.reserve(_wall_raw_points.size());

    const int window = 3;
    for (size_t i = 0; i < _wall_raw_points.size(); i++) {
        float sum_x = 0, sum_y = 0;
        int count = 0;

        for (int j = -window; j <= window; j++) {
            int idx = static_cast<int>(i) + j;
            if (idx >= 0 && idx < static_cast<int>(_wall_raw_points.size())) {
                sum_x += _wall_raw_points[idx].x();
                sum_y += _wall_raw_points[idx].y();
                count++;
            }
        }
        smoothed.push_back(Eigen::Vector2f(sum_x / count, sum_y / count));
    }

    // Simplify wall by merging line segments with similar angles (Ramer-Douglas-Peucker style)
    std::vector<Eigen::Vector2f> simplified;
    simplified.push_back(smoothed.front());

    const float angle_threshold = 0.3f; // ~17 degrees - aggressive merging

    for (size_t i = 1; i < smoothed.size(); i++) {
        if (simplified.size() < 2) {
            simplified.push_back(smoothed[i]);
            continue;
        }

        // Get direction of current segment (from second-to-last to last simplified point)
        Eigen::Vector2f prev_dir = simplified.back() - simplified[simplified.size() - 2];
        prev_dir.normalize();

        // Get direction to new point
        Eigen::Vector2f new_dir = smoothed[i] - simplified.back();
        float new_len = new_dir.norm();
        if (new_len < 1e-6f)
            continue;
        new_dir.normalize();

        // Check angle between directions
        float dot = prev_dir.dot(new_dir);
        dot = std::clamp(dot, -1.0f, 1.0f);
        float angle = std::acos(dot);

        if (angle < angle_threshold) {
            // Similar direction - merge by moving last point
            simplified.back() = smoothed[i];
        } else {
            // Different direction - add new point
            simplified.push_back(smoothed[i]);
        }
    }

    // Only add wall if it has at least 2 points
    if (simplified.size() >= 2) {
        // Update or create current wall
        if (_walls.empty() || _wall_raw_points.size() == 1) {
            _walls.push_back({simplified});
        } else {
            _walls.back().points = simplified;
        }
    }
}

// Helper: check if two line segments intersect
static bool segments_intersect(const Eigen::Vector2f& p1, const Eigen::Vector2f& p2, const Eigen::Vector2f& p3, const Eigen::Vector2f& p4) {
    Eigen::Vector2f d1 = p2 - p1;
    Eigen::Vector2f d2 = p4 - p3;

    float cross = d1.x() * d2.y() - d1.y() * d2.x();
    if (std::abs(cross) < 1e-10f)
        return false; // Parallel

    Eigen::Vector2f d3 = p3 - p1;
    float t = (d3.x() * d2.y() - d3.y() * d2.x()) / cross;
    float u = (d3.x() * d1.y() - d3.y() * d1.x()) / cross;

    // Intersection within segments (with small epsilon to avoid endpoint issues)
    return t > 0.001f && t < 0.999f && u > 0.001f && u < 0.999f;
}

bool App::is_landmark_occluded_by_walls(const Eigen::Vector2f& camera_pos, const Eigen::Vector2f& landmark) const {
    for (const auto& wall : _walls) {
        for (size_t i = 0; i + 1 < wall.points.size(); i++) {
            if (segments_intersect(camera_pos, landmark, wall.points[i], wall.points[i + 1])) {
                return true;
            }
        }
    }
    return false;
}

std::vector<Eigen::Vector2f> App::filter_visible_landmarks(const Eigen::Vector2f& camera_pos) const {
    std::vector<Eigen::Vector2f> visible;
    visible.reserve(_landmarks.size());
    for (const auto& lm : _landmarks) {
        if (!is_landmark_occluded_by_walls(camera_pos, lm)) {
            visible.push_back(lm);
        }
    }
    return visible;
}

void App::estimate() {
    _est_pos.clear();
    if (!_gt_trajectory.is_valid())
        return;

    size_t num_poses = _gt_trajectory.num_poses();
    _est_pos.resize(num_poses);
    for (size_t i = 0; i < num_poses; i++) {
        Eigen::Vector2f pos = _gt_trajectory.position(static_cast<float>(i));
        _est_pos[i] = pos + Eigen::Vector2f(0.0f, 10.0f);
    }
}

bool App::render_config() {
    bool updated = false;
    if (ImGui::CollapsingHeader("Config", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Indent();
        ImGui::PushItemWidth(100);

        ImGui::Text("Camera Config");
        int cam_width = _camera.width();
        float cam_fov_deg = _camera.fov() * 180.0f / M_PI;
        float cam_noise_std = _camera.noise_std();
        if (ImGui::DragInt("Width (px)", &cam_width)) {
            _camera.set_width(cam_width);
            updated = true;
        }
        if (ImGui::DragFloat("FOV (deg)", &cam_fov_deg)) {
            _camera.set_fov(cam_fov_deg * M_PI / 180.0f);
            updated = true;
        }
        if (ImGui::DragFloat("Camera noise std", &cam_noise_std, 0.1f, 0.0f, 10.0f)) {
            _camera.set_noise_std(cam_noise_std);
            updated = true;
        }
        ImGui::Spacing();

        ImGui::Text("IMU Config");
        Eigen::Vector2f acc_bias = _imu.acc_bias();
        Eigen::Vector2f acc_noise_std = _imu.acc_noise_std();
        float gyr_bias = _imu.gyr_bias();
        float gyr_noise_std = _imu.gyr_noise_std();
        if (ImGui::DragFloat2("Accel bias", acc_bias.data(), 0.01f, -1.0f, 1.0f)) {
            _imu.set_acc_bias(acc_bias);
            updated = true;
        }
        if (ImGui::DragFloat2("Accel noise std", acc_noise_std.data(), 0.001f, 0.0f, 1.0f)) {
            _imu.set_acc_noise_std(acc_noise_std);
            updated = true;
        }
        if (ImGui::DragFloat("Gyro bias", &gyr_bias, 0.001f, -0.1f, 0.1f)) {
            _imu.set_gyr_bias(gyr_bias);
            updated = true;
        }
        if (ImGui::DragFloat("Gyro noise std", &gyr_noise_std, 0.0001f, 0.0f, 0.1f)) {
            _imu.set_gyr_noise_std(gyr_noise_std);
            updated = true;
        }

        ImGui::PopItemWidth();
        ImGui::Unindent();
    }
    return updated;
}

void App::render_world_editor() {
    static bool trajectory_drag_started = false;
    static ImVec2 trajectory_drag_start_pos;
    static bool landmark_click_started = false;
    static ImVec2 landmark_click_start_pos;
    static bool wall_drag_started = false;
    static ImVec2 wall_drag_start_pos;

    if (ImGui::CollapsingHeader("World Editor", ImGuiTreeNodeFlags_DefaultOpen)) {
        // World preset selector
        const char* preset_names[] = {"Custom", "A Square's House", "Visit from the Sphere", "Hall of the Council"};
        int current_preset_idx = static_cast<int>(_current_preset);
        ImGui::SetNextItemWidth(200);
        if (ImGui::Combo("World Preset", &current_preset_idx, preset_names, IM_ARRAYSIZE(preset_names))) {
            load_world_preset(static_cast<WorldPreset>(current_preset_idx));
        }
        ImGui::Separator();

        ImGui::TextWrapped("Ctrl+Click: Add new landmark | Ctrl+Drag: Draw trajectory");
        ImGui::SameLine();
        if (ImGui::Button("Clear trajectory")) {
            _gt_pose_raw.clear();
            _gt_trajectory = Trajectory2D();
            _current_preset = WorldPreset::Custom;
        }

        ImGui::TextWrapped("Right-click landmark: Delete");
        ImGui::SameLine();
        if (ImGui::Button("Clear all landmarks")) {
            _landmarks.clear();
            _current_preset = WorldPreset::Custom;
        }

        ImGui::TextWrapped("Shift+Drag: Draw wall");
        ImGui::SameLine();
        if (ImGui::Button("Clear all walls")) {
            _walls.clear();
            _wall_raw_points.clear();
            _current_preset = WorldPreset::Custom;
        }

        // Disable plot panning when Ctrl or Shift is held (we're drawing)
        ImPlotFlags plot_flags = ImPlotFlags_Equal;
        if (ImGui::GetIO().KeyCtrl || ImGui::GetIO().KeyShift) {
            plot_flags |= ImPlotFlags_NoInputs;
        }

        if (ImPlot::BeginPlot("##WorldEditor", ImVec2(-1, 400), plot_flags)) {
            ImPlot::SetupAxes("X (m)", "Y (m)");

            // Get plot limits and mouse position
            ImPlotPoint mouse = ImPlot::GetPlotMousePos();
            bool is_hovered = ImPlot::IsPlotHovered();

            // Handle Ctrl+click to add landmark or Ctrl+drag to draw trajectory
            if (is_hovered && ImGui::GetIO().KeyCtrl) {
                if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                    trajectory_drag_started = true;
                    trajectory_drag_start_pos = ImGui::GetMousePos();
                    landmark_click_started = true;
                    landmark_click_start_pos = ImGui::GetMousePos();
                }

                if (trajectory_drag_started && ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
                    ImVec2 current_pos = ImGui::GetMousePos();
                    float drag_dist = std::sqrt(std::pow(current_pos.x - trajectory_drag_start_pos.x, 2) +
                                                std::pow(current_pos.y - trajectory_drag_start_pos.y, 2));

                    // If dragged more than threshold, switch to trajectory drawing mode
                    if (drag_dist > 5.0f) {
                        // Clear trajectory on first drag movement
                        if (landmark_click_started) {
                            _gt_pose_raw.clear();
                            _gt_trajectory = Trajectory2D();
                            landmark_click_started = false;
                            _current_preset = WorldPreset::Custom;
                        }

                        bool should_add = true;
                        if (!_gt_pose_raw.empty()) {
                            ImPlotPoint last_plot(_gt_pose_raw.back().x(), _gt_pose_raw.back().y());
                            ImVec2 last_px = ImPlot::PlotToPixels(last_plot);
                            ImVec2 mouse_px = ImPlot::PlotToPixels(mouse);
                            float dist = std::sqrt(std::pow(mouse_px.x - last_px.x, 2) + std::pow(mouse_px.y - last_px.y, 2));
                            should_add = dist > 1.0f;
                        }
                        if (should_add) {
                            // Compute orientation from previous pose
                            float orientation = 0.0f;
                            if (!_gt_pose_raw.empty()) {
                                Eigen::Vector2f prev(_gt_pose_raw.back().x(), _gt_pose_raw.back().y());
                                Eigen::Vector2f curr(mouse.x, mouse.y);
                                Eigen::Vector2f dir = curr - prev;
                                orientation = std::atan2(dir.y(), dir.x());
                            }
                            _gt_pose_raw.push_back(Eigen::Vector3f(mouse.x, mouse.y, orientation));
                            // Rebuild trajectory while drawing for live preview
                            build_trajectory_from_raw_poses();
                        }
                    }
                }

                // On release, if we didn't drag much, add a landmark
                if (landmark_click_started && ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
                    ImVec2 current_pos = ImGui::GetMousePos();
                    float dist =
                        std::sqrt(std::pow(current_pos.x - landmark_click_start_pos.x, 2) + std::pow(current_pos.y - landmark_click_start_pos.y, 2));
                    if (dist < 5.0f) {
                        _landmarks.push_back(Eigen::Vector2f(mouse.x, mouse.y));
                        _current_preset = WorldPreset::Custom;
                    }
                }
            }

            // Reset tracking on mouse release
            if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
                trajectory_drag_started = false;
                landmark_click_started = false;
                if (wall_drag_started) {
                    wall_drag_started = false;
                    _wall_raw_points.clear(); // Clear raw points after wall is finalized
                }
            }

            // Handle Shift+drag to draw walls
            if (is_hovered && ImGui::GetIO().KeyShift && !ImGui::GetIO().KeyCtrl) {
                if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                    wall_drag_started = true;
                    wall_drag_start_pos = ImGui::GetMousePos();
                    _wall_raw_points.clear();
                }

                if (wall_drag_started && ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
                    ImVec2 current_pos = ImGui::GetMousePos();
                    float drag_dist =
                        std::sqrt(std::pow(current_pos.x - wall_drag_start_pos.x, 2) + std::pow(current_pos.y - wall_drag_start_pos.y, 2));

                    if (drag_dist > 3.0f) {
                        // Start a new wall if this is the first point
                        if (_wall_raw_points.empty()) {
                            _walls.push_back(Wall{});
                            _current_preset = WorldPreset::Custom;
                        }

                        bool should_add = true;
                        if (!_wall_raw_points.empty()) {
                            ImPlotPoint last_plot(_wall_raw_points.back().x(), _wall_raw_points.back().y());
                            ImVec2 last_px = ImPlot::PlotToPixels(last_plot);
                            ImVec2 mouse_px_check = ImPlot::PlotToPixels(mouse);
                            float dist = std::sqrt(std::pow(mouse_px_check.x - last_px.x, 2) + std::pow(mouse_px_check.y - last_px.y, 2));
                            should_add = dist > 1.0f;
                        }
                        if (should_add) {
                            _wall_raw_points.push_back(Eigen::Vector2f(mouse.x, mouse.y));
                            build_wall_from_raw_points();
                        }
                    }
                }
            }

            // Find closest point (landmark or GT) for tooltip
            ImVec2 mouse_px = ImGui::GetMousePos();
            float closest_dist = 5.0f;
            int closest_landmark = -1;
            int closest_gt = -1;

            for (size_t i = 0; i < _landmarks.size(); i++) {
                ImVec2 lm_px = ImPlot::PlotToPixels(ImPlotPoint(_landmarks[i].x(), _landmarks[i].y()));
                float dist = std::sqrt(std::pow(mouse_px.x - lm_px.x, 2) + std::pow(mouse_px.y - lm_px.y, 2));
                if (dist < closest_dist) {
                    closest_dist = dist;
                    closest_landmark = static_cast<int>(i);
                    closest_gt = -1;
                }
            }

            // Check proximity to trajectory poses
            if (_gt_trajectory.is_valid()) {
                for (size_t i = 0; i < _gt_trajectory.num_poses(); i++) {
                    Eigen::Vector2f pos = _gt_trajectory.position(static_cast<float>(i));
                    ImVec2 gt_px = ImPlot::PlotToPixels(ImPlotPoint(pos.x(), pos.y()));
                    float dist = std::sqrt(std::pow(mouse_px.x - gt_px.x, 2) + std::pow(mouse_px.y - gt_px.y, 2));
                    if (dist < closest_dist) {
                        closest_dist = dist;
                        closest_gt = static_cast<int>(i);
                        closest_landmark = -1;
                    }
                }
            }

            // Render trajectory
            if (_gt_trajectory.is_valid()) {
                plot_2d_trajectory("Trajectory", _gt_trajectory, Color::Green());
            }

            // While drawing, show camera at last pose; otherwise show tooltip for closest point
            bool is_drawing = trajectory_drag_started && !landmark_click_started;
            if (is_drawing && _gt_trajectory.is_valid()) {
                // Show camera at last pose while drawing
                float last_t = _gt_trajectory.max_t();
                Eigen::Vector3f pose = _gt_trajectory.pose(last_t);
                Eigen::Vector2f pos(pose.x(), pose.y());
                // Filter landmarks by wall occlusion, keeping track of original indices
                std::vector<LandmarkObservation> observations;
                for (size_t i = 0; i < _landmarks.size(); i++) {
                    if (!is_landmark_occluded_by_walls(pos, _landmarks[i])) {
                        auto u = _camera.project(pose, _landmarks[i]);
                        if (u.has_value()) {
                            observations.push_back({u.value(), i});
                        }
                    }
                }
                plot_2d_camera_frustum("##DrawingCamera", pos, pose.z(), _camera.fov(), 1.0f, Color::Blue());
                plot_2d_camera_rays("##DrawingRays", pos, _landmarks, observations, 1.0f);
                plot_2d_camera_observations("##DrawingObs", pos, pose.z(), _camera, observations);
            } else {
                // Show tooltip for closest point and camera preview for poses
                if (closest_landmark >= 0) {
                    // Count observations for this landmark (considering wall occlusion)
                    int obs_count = 0;
                    if (_gt_trajectory.is_valid()) {
                        int num_poses = static_cast<int>(_gt_trajectory.max_t()) + 1;
                        for (int t = 0; t < num_poses; t++) {
                            Eigen::Vector3f pose = _gt_trajectory.pose(static_cast<float>(t));
                            Eigen::Vector2f pos(pose.x(), pose.y());
                            // Check wall occlusion before projecting
                            if (is_landmark_occluded_by_walls(pos, _landmarks[closest_landmark]))
                                continue;
                            auto u = _camera.project(pose, _landmarks[closest_landmark]);
                            if (u.has_value()) {
                                obs_count++;
                                // Only show observation for the hovered landmark
                                std::vector<LandmarkObservation> single_obs = {{u.value(), static_cast<size_t>(closest_landmark)}};
                                plot_2d_camera_frustum("##LandmarkHoverCamera", pos, pose.z(), _camera.fov(), 1.0f, Color::Blue());
                                plot_2d_camera_rays("##LandmarkHoverRays", pos, _landmarks, single_obs, 1.0f);
                                plot_2d_camera_observations("##LandmarkHoverObs", pos, pose.z(), _camera, single_obs);
                            }
                        }
                    }
                    ImGui::SetTooltip("Landmark %d\nPos: (%.2f, %.2f)\nObservations: %d", closest_landmark, _landmarks[closest_landmark].x(),
                                      _landmarks[closest_landmark].y(), obs_count);
                } else if (closest_gt >= 0 && _gt_trajectory.is_valid()) {
                    Eigen::Vector3f pose = _gt_trajectory.pose(static_cast<float>(closest_gt));
                    // Draw camera frustum with projected landmarks (filtered by walls)
                    Eigen::Vector2f pos(pose.x(), pose.y());
                    std::vector<LandmarkObservation> observations;
                    for (size_t i = 0; i < _landmarks.size(); i++) {
                        if (!is_landmark_occluded_by_walls(pos, _landmarks[i])) {
                            auto u = _camera.project(pose, _landmarks[i]);
                            if (u.has_value()) {
                                observations.push_back({u.value(), i});
                            }
                        }
                    }
                    ImGui::SetTooltip("GT Pose %d\nPos: (%.2f, %.2f)\nOrientation: %.2f°\nObservations: %zu", closest_gt, pose.x(), pose.y(),
                                      pose.z() * 180.0f / M_PI, observations.size());
                    plot_2d_camera_frustum("##HoverCamera", pos, pose.z(), _camera.fov(), 1.0f, Color::Blue());
                    plot_2d_camera_rays("##HoverRays", pos, _landmarks, observations, 1.0f);
                    plot_2d_camera_observations("##HoverObs", pos, pose.z(), _camera, observations);
                }
            }

            // Render landmarks as draggable points with context menu
            int landmark_to_delete = -1;
            for (size_t i = 0; i < _landmarks.size(); i++) {
                double x = _landmarks[i].x();
                double y = _landmarks[i].y();
                Color lm_color = Color::Random(i);
                if (ImPlot::DragPoint(static_cast<int>(i), &x, &y, ImVec4(lm_color), 4.0f)) {
                    _landmarks[i] = Eigen::Vector2f(x, y);
                    _current_preset = WorldPreset::Custom;
                }

                // Check if mouse is hovering this landmark for context menu
                ImVec2 lm_px = ImPlot::PlotToPixels(ImPlotPoint(x, y));
                float dist = std::sqrt(std::pow(mouse_px.x - lm_px.x, 2) + std::pow(mouse_px.y - lm_px.y, 2));
                if (dist < 10.0f) {
                    if (ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
                        ImGui::OpenPopup(("LandmarkContext" + std::to_string(i)).c_str());
                    }
                }

                if (ImGui::BeginPopup(("LandmarkContext" + std::to_string(i)).c_str())) {
                    ImGui::Text("Landmark %zu", i);
                    ImGui::Separator();
                    if (ImGui::MenuItem("Delete")) {
                        landmark_to_delete = static_cast<int>(i);
                    }
                    ImGui::EndPopup();
                }
            }
            if (landmark_to_delete >= 0) {
                _landmarks.erase(_landmarks.begin() + landmark_to_delete);
                _current_preset = WorldPreset::Custom;
            }

            // Render walls (plain white)
            int wall_to_delete = -1;
            for (size_t w = 0; w < _walls.size(); w++) {
                const auto& wall = _walls[w];
                if (wall.points.size() >= 2) {
                    plot_2d_line("##Wall" + std::to_string(w), wall.points, Color::White(), 2.0f);

                    // Check if mouse is near any segment of this wall for context menu
                    for (size_t i = 0; i + 1 < wall.points.size(); i++) {
                        ImVec2 p1_px = ImPlot::PlotToPixels(ImPlotPoint(wall.points[i].x(), wall.points[i].y()));
                        ImVec2 p2_px = ImPlot::PlotToPixels(ImPlotPoint(wall.points[i + 1].x(), wall.points[i + 1].y()));

                        // Distance from mouse to line segment
                        float dx = p2_px.x - p1_px.x;
                        float dy = p2_px.y - p1_px.y;
                        float len_sq = dx * dx + dy * dy;
                        float t = 0.0f;
                        if (len_sq > 0) {
                            t = std::clamp(((mouse_px.x - p1_px.x) * dx + (mouse_px.y - p1_px.y) * dy) / len_sq, 0.0f, 1.0f);
                        }
                        float closest_x = p1_px.x + t * dx;
                        float closest_y = p1_px.y + t * dy;
                        float dist = std::sqrt(std::pow(mouse_px.x - closest_x, 2) + std::pow(mouse_px.y - closest_y, 2));

                        if (dist < 10.0f && ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
                            ImGui::OpenPopup(("WallContext" + std::to_string(w)).c_str());
                            break;
                        }
                    }
                }

                if (ImGui::BeginPopup(("WallContext" + std::to_string(w)).c_str())) {
                    ImGui::Text("Wall %zu (%zu segments)", w, wall.points.size() > 0 ? wall.points.size() - 1 : 0);
                    ImGui::Separator();
                    if (ImGui::MenuItem("Delete")) {
                        wall_to_delete = static_cast<int>(w);
                    }
                    ImGui::EndPopup();
                }
            }
            if (wall_to_delete >= 0) {
                _walls.erase(_walls.begin() + wall_to_delete);
                _current_preset = WorldPreset::Custom;
            }

            ImPlot::EndPlot();
        }
    }
}

void App::render_measurements() {
    if (ImGui::CollapsingHeader("Sensor Measurements")) {
        ImGui::Text("TODO");
    }
}

void App::render_perception_output() {
    if (ImGui::CollapsingHeader("Perception Output", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImPlot::BeginPlot("Position")) {
            if (_gt_trajectory.is_valid()) {
                plot_2d_trajectory("Ground-truth", _gt_trajectory, Color::Green());
            }
            plot_2d_path("Estimated", _est_pos, Color::Red());
            ImPlot::EndPlot();
        }
    }
}

void App::render_error_metrics() {
    if (ImGui::CollapsingHeader("Error Metrics")) {
        ImGui::Text("TODO");
    }
}
