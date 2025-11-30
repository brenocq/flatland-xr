// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#pragma once

#include <gui/ui_state.hpp>
#include <sensors/camera2d.hpp>
#include <sensors/imu2d.hpp>
#include <simulation/simulation.hpp>

namespace gui {

/// Panel for displaying sensor measurements
class MeasurementsPanel {
  public:
    MeasurementsPanel() = default;

    void set_ui_state(UIState::SharedPtr ui_state) { _ui_state = std::move(ui_state); }

    /// Render the panel
    void render(const simulation::SimulationResult& sim_result, const sensors::Camera2D& camera, const sensors::IMU2D& imu);

  private:
    UIState::SharedPtr _ui_state;
};

} // namespace gui
