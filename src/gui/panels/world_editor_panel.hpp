// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#pragma once

#include "imgui.h"
#include <Eigen/Dense>
#include <core/trajectory2d.hpp>
#include <core/types.hpp>
#include <gui/ui_state.hpp>
#include <sensors/camera2d.hpp>
#include <vector>
#include <world/world.hpp>

namespace gui {

/// Panel for editing the world (trajectory, landmarks, walls)
class WorldEditorPanel {
  public:
    WorldEditorPanel() = default;

    void set_ui_state(UIState::SharedPtr ui_state) { _ui_state = std::move(ui_state); }

    /// Render the panel. Returns true if the world changed.
    bool render(world::Preset& current_preset, std::vector<Eigen::Vector3f>& gt_pose_raw, core::Trajectory2D& gt_trajectory,
                std::vector<Eigen::Vector2f>& landmarks, std::vector<core::Wall>& walls, std::vector<Eigen::Vector2f>& wall_raw_points,
                const sensors::Camera2D& camera);

  private:
    UIState::SharedPtr _ui_state;

    // Drawing state
    bool _trajectory_drag_started = false;
    ImVec2 _trajectory_drag_start_pos = ImVec2(0, 0);
    bool _landmark_click_started = false;
    ImVec2 _landmark_click_start_pos = ImVec2(0, 0);
    bool _wall_drag_started = false;
    ImVec2 _wall_drag_start_pos = ImVec2(0, 0);
};

} // namespace gui
