// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include "imgui.h"
#include <gui/panels/error_metrics_panel.hpp>

namespace gui {

void ErrorMetricsPanel::render() {
    if (ImGui::CollapsingHeader("Error Metrics")) {
        ImGui::Text("TODO");
    }
}

} // namespace gui
