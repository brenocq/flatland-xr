// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include "imgui.h"
#include "implot.h"

#include <cmath>
#include <gui/app.hpp>
#include <gui/plot.hpp>

App::App() {}

void App::startup() {}

void App::shutdown() {}

void App::update() {
    // Docking
    ImGuiID dockId = ImGui::GetID("##DockSpace");
    ImGui::DockSpaceOverViewport(dockId, ImGui::GetMainViewport());
    ImGui::SetNextWindowDockID(dockId, ImGuiCond_Appearing);
    if (ImGui::Begin("Lineland XR")) {
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

        ImGui::Text("General");
        updated |= ImGui::DragInt("Num of steps", &_num_steps);
        updated |= ImGui::DragInt("Num of landmarks", &_num_landmarks);
        ImGui::Spacing();

        ImGui::Text("Camera Config");
        updated |= ImGui::DragInt("Width (px)", &_cam_width);
        updated |= ImGui::DragFloat("FOV (deg)", &_cam_fov);
        updated |= ImGui::DragFloat("Camera noise std", &_cam_noise_std);
        ImGui::Spacing();

        ImGui::Text("IMU Config");
        updated |= ImGui::DragFloat("Accel noise std", &_acc_noise_std);
        updated |= ImGui::DragFloat("Gyro noise std", &_gyr_noise_std);

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

    if (ImGui::CollapsingHeader("World Editor", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::TextWrapped("Ctrl+Click: Add new landmark | Ctrl+Drag: Draw trajectory");
        ImGui::SameLine();
        if (ImGui::Button("Clear trajectory")) {
            _gt_pose_raw.clear();
            _gt_trajectory = Trajectory2D();
        }

        ImGui::TextWrapped("Right-click landmark: Delete");
        ImGui::SameLine();
        if (ImGui::Button("Clear all landmarks"))
            _landmarks.clear();

        if (ImPlot::BeginPlot("##WorldEditor", ImVec2(-1, 400), ImPlotFlags_Equal)) {
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
                    }
                }
            }

            // Reset tracking on mouse release
            if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
                trajectory_drag_started = false;
                landmark_click_started = false;
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
                float orientation = pose.z();
                float fov_rad = _cam_fov * M_PI / 180.0f;
                plot_2d_camera("##DrawingCamera", pos, orientation, fov_rad, 1.0f, Color::Red());
            } else {
                // Show tooltip for closest point and camera preview for poses
                if (closest_landmark >= 0) {
                    ImGui::SetTooltip("Landmark %d\nPos: (%.2f, %.2f)", closest_landmark, _landmarks[closest_landmark].x(),
                                      _landmarks[closest_landmark].y());
                } else if (closest_gt >= 0 && _gt_trajectory.is_valid()) {
                    Eigen::Vector3f pose = _gt_trajectory.pose(static_cast<float>(closest_gt));
                    ImGui::SetTooltip("GT Pose %d\nPos: (%.2f, %.2f)\nOrientation: %.2f°", closest_gt, pose.x(), pose.y(), pose.z() * 180.0f / M_PI);
                    // Draw camera frustum at hovered pose
                    Eigen::Vector2f pos(pose.x(), pose.y());
                    float orientation = pose.z();
                    float fov_rad = _cam_fov * M_PI / 180.0f;
                    plot_2d_camera("##HoverCamera", pos, orientation, fov_rad, 1.0f, Color::Red());
                }
            }

            // Render landmarks as draggable points with context menu
            int landmark_to_delete = -1;
            for (size_t i = 0; i < _landmarks.size(); i++) {
                double x = _landmarks[i].x();
                double y = _landmarks[i].y();
                if (ImPlot::DragPoint(static_cast<int>(i), &x, &y, ImVec4(1, 0.5f, 0, 1), 4.0f)) {
                    _landmarks[i] = Eigen::Vector2f(x, y);
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
