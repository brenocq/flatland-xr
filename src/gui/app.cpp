// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include "imgui.h"
#include "implot.h"

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
    _gt_pose.resize(_num_steps);
    _est_pos.clear();

    _gt_pose[0] = {0.0f, 0.0f, 0.0f};
    for (size_t i = 1; i < _num_steps; i++) {
        Eigen::Vector2f prev(_gt_pose[i - 1].x(), _gt_pose[i - 1].y());
        Eigen::Vector2f curr = prev + Eigen::Vector2f(1.0f, 0.0f);
        float orientation = std::atan2(curr.y() - prev.y(), curr.x() - prev.x());
        _gt_pose[i] = Eigen::Vector3f(curr.x(), curr.y(), orientation);
    }
}

void App::estimate() {
    _est_pos.resize(_num_steps);
    _est_pos[0] = {0.0f, 10.0f};
    for (size_t i = 1; i < _num_steps; i++) {
        _est_pos[i] = Eigen::Vector2f(_gt_pose[i].x(), _gt_pose[i].y()) + Eigen::Vector2f(0.0f, 10.0f);
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
        if (ImGui::Button("Clear trajectory"))
            _gt_pose.clear();

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
                            _gt_pose.clear();
                            landmark_click_started = false;
                        }

                        bool should_add = true;
                        if (!_gt_pose.empty()) {
                            ImPlotPoint last_plot(_gt_pose.back().x(), _gt_pose.back().y());
                            ImVec2 last_px = ImPlot::PlotToPixels(last_plot);
                            ImVec2 mouse_px = ImPlot::PlotToPixels(mouse);
                            float dist = std::sqrt(std::pow(mouse_px.x - last_px.x, 2) + std::pow(mouse_px.y - last_px.y, 2));
                            should_add = dist > 1.0f;
                        }
                        if (should_add) {
                            // Compute orientation from previous pose
                            float orientation = 0.0f;
                            if (!_gt_pose.empty()) {
                                Eigen::Vector2f prev(_gt_pose.back().x(), _gt_pose.back().y());
                                Eigen::Vector2f curr(mouse.x, mouse.y);
                                Eigen::Vector2f dir = curr - prev;
                                orientation = std::atan2(dir.y(), dir.x());
                            }
                            _gt_pose.push_back(Eigen::Vector3f(mouse.x, mouse.y, orientation));
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

            for (size_t i = 0; i < _gt_pose.size(); i++) {
                ImVec2 gt_px = ImPlot::PlotToPixels(ImPlotPoint(_gt_pose[i].x(), _gt_pose[i].y()));
                float dist = std::sqrt(std::pow(mouse_px.x - gt_px.x, 2) + std::pow(mouse_px.y - gt_px.y, 2));
                if (dist < closest_dist) {
                    closest_dist = dist;
                    closest_gt = static_cast<int>(i);
                    closest_landmark = -1;
                }
            }

            // Render trajectory using plot_2d_poses
            if (!_gt_pose.empty()) {
                plot_2d_poses("Trajectory", _gt_pose, Color::Green());
            }

            // Show tooltip for closest point and camera preview for poses
            if (closest_landmark >= 0) {
                ImGui::SetTooltip("Landmark %d\nPos: (%.2f, %.2f)", closest_landmark, _landmarks[closest_landmark].x(),
                                  _landmarks[closest_landmark].y());
            } else if (closest_gt >= 0) {
                ImGui::SetTooltip("GT Pose %d\nPos: (%.2f, %.2f)\nOrientation: %.2f°", closest_gt, _gt_pose[closest_gt].x(), _gt_pose[closest_gt].y(),
                                  _gt_pose[closest_gt].z() * 180.0f / M_PI);
                // Draw camera frustum at hovered pose
                Eigen::Vector2f pos(_gt_pose[closest_gt].x(), _gt_pose[closest_gt].y());
                float orientation = _gt_pose[closest_gt].z();
                float fov_rad = _cam_fov * M_PI / 180.0f;
                plot_2d_camera("##HoverCamera", pos, orientation, fov_rad, 1.0f, Color::Red());
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
            plot_2d_poses("Ground-truth", _gt_pose, Color::Green());
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
