// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include <gui/app.hpp>
#include "imgui.h"
#include "implot.h"

App::App() {}

void App::startup() {}
void App::shutdown() {}

void App::update() {
    // Docking
    ImGuiID dockId = ImGui::GetID("##DockSpace");
    ImGui::DockSpaceOverViewport(dockId, ImGui::GetMainViewport());
    ImGui::SetNextWindowDockID(dockId, ImGuiCond_Appearing);
    ImGui::Begin("MSCKF Playground");
    {
        ImGui::Text("Some cool MSCKF is about to show up here");
    }
    ImGui::End();

    ImGui::ShowDemoWindow();
    ImPlot::ShowDemoWindow();
}
