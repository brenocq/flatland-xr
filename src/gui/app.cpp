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
    if (ImGui::Begin("MSCKF Playground")) {
        render();
    }
    ImGui::End();

    // ImGui::ShowDemoWindow();
    // ImPlot::ShowDemoWindow();
}

void App::render() {
   ImGui::Text("Some cool MSCKF is about to show up here");

   const size_t NUM_STEPS = 100;
   std::vector<Eigen::Vector2d> est_pos(NUM_STEPS);
   std::vector<Eigen::Vector2d> gt_pos(NUM_STEPS);

   gt_pos[0] = {0.0, 0.0};
   est_pos[0] = {0.0, 0.0};
   for (size_t i = 1; i < NUM_STEPS; i++) {
       gt_pos[i] = gt_pos[i-1] + Eigen::Vector2d(1.0, 0.0);
       est_pos[i] = gt_pos[i];
   }

   if (ImPlot::BeginPlot("Position")) {
       plot_2d_path("Ground-truth", gt_pos);
       plot_2d_path("Estimated", est_pos);
       ImPlot::EndPlot();
   }
}
