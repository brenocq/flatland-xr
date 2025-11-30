// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include "error_metrics_panel.hpp"
#include "imgui.h"

namespace gui {

void ErrorMetricsPanel::render() {
    if (ImGui::CollapsingHeader("Error Metrics")) {
        ImGui::Text("TODO");
    }
}

} // namespace gui
