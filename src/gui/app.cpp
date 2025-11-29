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
   ImGui::Text("Some cool MSCKF is about to show up here");

   if (render_config() || _first_render) {
        _first_render = false;
        simulate();
    }

   const size_t NUM_STEPS = 100;
   std::vector<Eigen::Vector2d> est_pos(NUM_STEPS);
   std::vector<Eigen::Vector2d> gt_pos(NUM_STEPS);

   gt_pos[0] = {0.0, 0.0};
   est_pos[0] = {0.0, 0.0};
   for (size_t i = 1; i < NUM_STEPS; i++) {
       gt_pos[i] = gt_pos[i-1] + Eigen::Vector2d(1.0, 0.0);
       est_pos[i] = gt_pos[i] + Eigen::Vector2d(0.0, 10.0);
   }


   if (ImPlot::BeginPlot("Position")) {
       plot_2d_path("Ground-truth", gt_pos, Color::Green());
       plot_2d_path("Estimated", est_pos, Color::Red());
       ImPlot::EndPlot();
   }
}

bool App::render_config() {
    bool updated = false;
    if(ImGui::CollapsingHeader("Config", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Indent();
        ImGui::PushItemWidth(100);
        updated |= ImGui::DragInt("Num of steps", &_num_steps);
        updated |= ImGui::DragInt("Num of landmarks", &_num_landmarks);

        updated |= ImGui::DragInt("Camera width (px)", &_cam_width);
        updated |= ImGui::DragFloat("Camera FOV (deg)", &_cam_fov);

        updated |= ImGui::DragFloat("IMU acc noise std", &_acc_noise_std);
        updated |= ImGui::DragFloat("IMU gyr noise std", &_gyr_noise_std);
        updated |= ImGui::DragFloat("Camera noise std", &_cam_noise_std);
        ImGui::PopItemWidth();
        ImGui::Unindent();
    }
    return updated;
}

void App::simulate() {
    std::cout << "Simulate!\n";
}

void App::estimate() {

}
