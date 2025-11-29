// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include "imgui.h"
#include "implot.h"

#include <gui/app.hpp>
#include <gui/plot.hpp>
#include <iostream>

App::App() {}

void App::startup() {}

void App::shutdown() {}

void App::update() {
    // Docking
    ImGuiID dockId = ImGui::GetID("##DockSpace");
    ImGui::DockSpaceOverViewport(dockId, ImGui::GetMainViewport());
    ImGui::SetNextWindowDockID(dockId, ImGuiCond_Appearing);
    if (ImGui::Begin("MSCKF Playground")) {
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

    if (ImGui::CollapsingHeader("Output", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImPlot::BeginPlot("Position")) {
            plot_2d_path("Ground-truth", _gt_pos, Color::Green());
            plot_2d_path("Estimated", _est_pos, Color::Red());
            ImPlot::EndPlot();
        }
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
