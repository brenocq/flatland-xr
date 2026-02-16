// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include "imgui.h"
#include "imgui_internal.h"
#include "implot.h"

#include <gui/app.hpp>
#include <gui/widgets/text.hpp>

namespace gui {
using namespace gui::widgets;

App::App() {}

void App::startup() {}

void App::shutdown() {}

void App::update() {
    // Menu bar
    render_menu_bar();

    // Docking - Create a dockspace over the main viewport
    ImGuiID dockspace_id = ImGui::DockSpaceOverViewport(0, ImGui::GetMainViewport());

    // Setup initial layout on first frame
    if (_first_render) {
        setup_docking_layout(dockspace_id);
    }

    // Render benches (each creates its own window)
    _xr_bench.render();
    _gravity_bench.render();
    _fluid_bench.render();

    // Demo windows
    if (_show_imgui_demo) {
        ImGui::ShowDemoWindow(&_show_imgui_demo);
    }
    if (_show_implot_demo) {
        ImPlot::ShowDemoWindow(&_show_implot_demo);
    }
    if (_show_about) {
        render_about_window();
    }

    _first_render = false;
}

void App::setup_docking_layout(ImGuiID dockspace_id) {
    ImGui::DockBuilderRemoveNode(dockspace_id);
    ImGui::DockBuilderAddNode(dockspace_id, ImGuiDockNodeFlags_DockSpace);
    ImGui::DockBuilderSetNodeSize(dockspace_id, ImGui::GetMainViewport()->WorkSize);

    // Dock bench windows to the main viewport
    ImGui::DockBuilderDockWindow("XR Bench", dockspace_id);
    ImGui::DockBuilderDockWindow("Gravity Bench", dockspace_id);
    ImGui::DockBuilderDockWindow("Fluid Bench", dockspace_id);

    ImGui::DockBuilderFinish(dockspace_id);
}

void App::render_menu_bar() {
    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("Windows")) {
            ImGui::MenuItem("ImGui Demo", nullptr, &_show_imgui_demo);
            ImGui::MenuItem("ImPlot Demo", nullptr, &_show_implot_demo);
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Help")) {
            ImGui::MenuItem("About Flatland XR", nullptr, &_show_about);
            ImGui::EndMenu();
        }
        ImGui::EndMainMenuBar();
    }
}

void App::render_about_window() {
    if (!ImGui::Begin("About Flatland XR", &_show_about, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::End();
        return;
    }

    widgets::Text("Flatland XR v0.1");
    ImGui::Separator();

    widgets::TextLinkOpenURL("Homepage", "https://github.com/brenocq/flatland-xr");
    ImGui::SameLine();
    widgets::TextLinkOpenURL("Issues", "https://github.com/brenocq/flatland-xr/issues");
    ImGui::SameLine();
    widgets::TextLinkOpenURL("Releases", "https://github.com/brenocq/flatland-xr/releases");
    ImGui::SameLine();
    widgets::TextLinkOpenURL("Sponsor", "https://github.com/sponsors/brenocq");

    ImGui::Separator();
    widgets::Text("(c) 2025 Breno Cunha Queiroz");
    widgets::Text("Flatland XR is licensed under the MIT License.");
    widgets::Text("If you enjoy Flatland XR, please consider sponsoring the project.");

    ImGui::End();
}

} // namespace gui
