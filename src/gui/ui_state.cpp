// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include "implot.h"
#include <cmath>
#include <gui/plot.hpp>
#include <gui/ui_state.hpp>

namespace gui {

void UIState::handle_hovered_time(size_t max_time_idx) {
    // Update hovered time if plot is hovered
    if (ImPlot::IsPlotHovered()) {
        ImPlotPoint mouse = ImPlot::GetPlotMousePos();
        int idx = static_cast<int>(std::round(mouse.x));
        if (idx >= 0 && idx < static_cast<int>(max_time_idx)) {
            hovered_time_index = idx;
        }
    }

    // Draw highlight line if we have a valid hovered time
    if (has_hovered_time()) {
        plot_time_highlight(static_cast<size_t>(hovered_time_index));
    }
}

void UIState::handle_hovered_pose(const std::vector<Eigen::Vector3f>& poses) {
    // Update hovered time if plot is hovered and mouse is close to a pose
    if (ImPlot::IsPlotHovered()) {
        ImPlotPoint mouse = ImPlot::GetPlotMousePos();
        ImVec2 mouse_px = ImPlot::PlotToPixels(mouse);

        float closest_dist = 10.0f;
        int closest_idx = -1;

        for (size_t i = 0; i < poses.size(); i++) {
            ImVec2 pose_px = ImPlot::PlotToPixels(ImPlotPoint(poses[i].x(), poses[i].y()));
            float dist = mouse_px.distance(pose_px);
            if (dist < closest_dist) {
                closest_dist = dist;
                closest_idx = static_cast<int>(i);
            }
        }

        if (closest_idx >= 0) {
            hovered_time_index = closest_idx;
        }
    }

    // Draw highlight circle if we have a valid hovered time
    if (has_hovered_time() && hovered_time_index < static_cast<int>(poses.size())) {
        plot_pose_highlight(poses[hovered_time_index]);
    }
}

} // namespace gui
