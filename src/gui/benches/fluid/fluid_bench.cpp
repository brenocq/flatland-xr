// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include "imgui.h"
#include "implot.h"
#include <gui/benches/fluid/fluid_bench.hpp>
#include <gui/color.hpp>

namespace gui {

FluidBench::FluidBench() : Bench("Fluid") {}

void FluidBench::render_config_panel() {
    ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Configuration");
    ImGui::Separator();

    ImGui::Text("Coming soon...");
}

void FluidBench::render() {
    if (!ImGui::Begin("Fluid Bench")) {
        ImGui::End();
        return;
    }

    // Left side: Main visualization
    {
        ImGui::BeginChild("left pane", ImVec2(ImGui::GetContentRegionAvail().x * 0.75f, 0), ImGuiChildFlags_None);

        if (ImPlot::BeginPlot("Fluid Simulation", ImVec2(-1, -1))) {
            ImPlot::SetupAxes("X", "Y");
            ImPlot::SetupAxisLimits(ImAxis_X1, 0.0, 10.0);
            ImPlot::SetupAxisLimits(ImAxis_Y1, 0.0, 10.0);

            // Empty plot for now

            ImPlot::EndPlot();
        }

        ImGui::EndChild();
    }

    ImGui::SameLine();

    // Right side: Config panel
    {
        ImGui::BeginChild("config pane", ImVec2(0, 0), ImGuiChildFlags_Borders);
        render_config_panel();
        ImGui::EndChild();
    }

    ImGui::End();
}

} // namespace gui
