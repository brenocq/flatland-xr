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

    /// Currently selected time index for inspection (-1 if none)
    int selected_time_index = -1;

    bool has_selected_time() const { return selected_time_index >= 0; }
    void reset_selection() { selected_time_index = -1; }

    /// Draw draggable time selector line (call inside active ImPlot context)
    /// Returns true if the time index was changed
    bool handle_time_selector(size_t max_time_idx);

    /// Handle click detection on poses to select time (call inside active ImPlot context)
    /// Returns true if a pose was clicked
    bool handle_pose_selection(const std::vector<Eigen::Vector3f>& poses);
};

} // namespace gui
