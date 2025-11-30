// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#pragma once

#include <Eigen/Dense>
#include <cstddef>
#include <memory>
#include <vector>

namespace gui {

/// Shared UI state for synchronized interactions across panels
struct UIState {
    using SharedPtr = std::shared_ptr<UIState>;

    /// Currently hovered time index (-1 if none)
    int hovered_time_index = -1;

    bool has_hovered_time() const { return hovered_time_index >= 0; }
    void reset_hover() { hovered_time_index = -1; }

    /// Handle hover detection and draw time highlight line (call inside active ImPlot context)
    void handle_hovered_time(size_t max_time_idx);

    /// Handle hover detection over poses and draw pose highlight (call inside active ImPlot context)
    void handle_hovered_pose(const std::vector<Eigen::Vector3f>& poses);
};

} // namespace gui
