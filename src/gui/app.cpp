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
    _gt_pos.resize(_num_steps);
    _est_pos.clear();

    _gt_pos[0] = {0.0f, 0.0f};
    for (size_t i = 1; i < _num_steps; i++) {
        _gt_pos[i] = _gt_pos[i - 1] + Eigen::Vector2f(1.0f, 0.0f);
    }
}

void App::estimate() {
    _est_pos.resize(_num_steps);
    _est_pos[0] = {0.0f, 10.0f};
    for (size_t i = 1; i < _num_steps; i++) {
        _est_pos[i] = _gt_pos[i] + Eigen::Vector2f(0.0f, 10.0f);
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
    static bool landmark_click_started = false;
    static ImVec2 landmark_click_start_pos;

    if (ImGui::CollapsingHeader("World Editor"), ImGuiTreeNodeFlags_DefaultOpen) {
        ImGui::TextWrapped("Ctrl+Drag: Draw ground truth trajectory (clears previous)");
        ImGui::TextWrapped("Click: Add new landmark");

        if (ImPlot::BeginPlot("##WorldEditor", ImVec2(-1, 400))) {
            ImPlot::SetupAxes("X (m)", "Y (m)");

            // Get plot limits and mouse position
            ImPlotPoint mouse = ImPlot::GetPlotMousePos();
            bool is_hovered = ImPlot::IsPlotHovered();

            // Handle Ctrl+drag to draw trajectory
            if (is_hovered && ImGui::GetIO().KeyCtrl) {
                // Clear trajectory when Ctrl+click starts
                if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                    _gt_pos.clear();
                }

                if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
                    bool should_add = true;
                    if (!_gt_pos.empty()) {
                        // Convert last position and current mouse to pixels
                        ImPlotPoint last_plot(_gt_pos.back().x(), _gt_pos.back().y());
                        ImVec2 last_px = ImPlot::PlotToPixels(last_plot);
                        ImVec2 mouse_px = ImPlot::PlotToPixels(mouse);
                        float dist = std::sqrt(std::pow(mouse_px.x - last_px.x, 2) + std::pow(mouse_px.y - last_px.y, 2));
                        should_add = dist > 1.0f;
                    }
                    if (should_add) {
                        _gt_pos.push_back(Eigen::Vector2f(mouse.x, mouse.y));
                    }
                }
            }

            // Handle click to add landmark (without Ctrl) - only if mouse didn't move much
            if (is_hovered && !ImGui::GetIO().KeyCtrl) {
                if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                    landmark_click_started = true;
                    landmark_click_start_pos = ImGui::GetMousePos();
                }
                if (landmark_click_started && ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
                    ImVec2 current_pos = ImGui::GetMousePos();
                    float dist =
                        std::sqrt(std::pow(current_pos.x - landmark_click_start_pos.x, 2) + std::pow(current_pos.y - landmark_click_start_pos.y, 2));
                    if (dist < 5.0f) {
                        _landmarks.push_back(Eigen::Vector2f(mouse.x, mouse.y));
                    }
                    landmark_click_started = false;
                }
            }

            // Reset click tracking if mouse released
            if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
                landmark_click_started = false;
            }

            // Render landmarks as draggable points
            for (size_t i = 0; i < _landmarks.size(); i++) {
                double x = _landmarks[i].x();
                double y = _landmarks[i].y();
                if (ImPlot::DragPoint(static_cast<int>(i), &x, &y, ImVec4(1, 0.5f, 0, 1), 4.0f)) {
                    _landmarks[i] = Eigen::Vector2f(x, y);
                }
            }

            // Render trajectory using plot_2d_path
            if (!_gt_pos.empty()) {
                plot_2d_path("Trajectory", _gt_pos, Color::Green());
            }

            ImPlot::EndPlot();
        }

        // Add clear buttons
        if (ImGui::Button("Clear Trajectory")) {
            _gt_pos.clear();
        }
        ImGui::SameLine();
        if (ImGui::Button("Clear Landmarks")) {
            _landmarks.clear();
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
            plot_2d_path("Ground-truth", _gt_pos, Color::Green());
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
