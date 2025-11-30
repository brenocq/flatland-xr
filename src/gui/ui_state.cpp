// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include "implot.h"
#include <cmath>
#include <gui/color.hpp>
#include <gui/plot.hpp>
#include <gui/ui_state.hpp>

namespace gui {

bool UIState::handle_time_selector(size_t max_time_idx) {
    if (max_time_idx == 0)
        return false;

    // Initialize to middle if not set
    if (selected_time_index < 0) {
        selected_time_index = static_cast<int>(max_time_idx) / 2;
    }

    // Clamp to valid range
    if (selected_time_index >= static_cast<int>(max_time_idx)) {
        selected_time_index = static_cast<int>(max_time_idx) - 1;
    }

    double time_value = static_cast<double>(selected_time_index);
    ImPlot::DragLineX(0, &time_value, ImVec4(Color::Red()), 2.0f);

    // Snap to nearest integer time index
    int new_index = static_cast<int>(std::round(time_value));
    new_index = std::max(0, std::min(new_index, static_cast<int>(max_time_idx) - 1));

    bool changed = (new_index != selected_time_index);
    selected_time_index = new_index;

    return changed;
}

bool UIState::handle_pose_selection(const std::vector<Eigen::Vector3f>& poses) {
    if (poses.empty())
        return false;

    bool pose_clicked = false;

    // Check if mouse is hovering over a pose
    if (ImPlot::IsPlotHovered()) {
        ImPlotPoint mouse = ImPlot::GetPlotMousePos();
        ImVec2 mouse_px = ImPlot::PlotToPixels(mouse);

        float closest_dist = 5.0f;
        int closest_idx = -1;

        for (size_t i = 0; i < poses.size(); i++) {
            ImVec2 pose_px = ImPlot::PlotToPixels(ImPlotPoint(poses[i].x(), poses[i].y()));
            float dist = mouse_px.distance(pose_px);
            if (dist < closest_dist) {
                closest_dist = dist;
                closest_idx = static_cast<int>(i);
            }
        }

        // If hovering over a pose, change cursor to hand
        if (closest_idx >= 0) {
            ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);

            // Check if user clicked
            if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                selected_time_index = closest_idx;
                pose_clicked = true;
            }
        }
    }

    // Draw highlight circle if we have a valid selected time
    if (has_selected_time() && selected_time_index < static_cast<int>(poses.size())) {
        plot_pose_highlight(poses[selected_time_index]);
    }

    return pose_clicked;
}

} // namespace gui
