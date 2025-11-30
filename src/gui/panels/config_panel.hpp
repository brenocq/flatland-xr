// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#pragma once

#include <gui/ui_state.hpp>
#include <sensors/camera2d.hpp>
#include <sensors/imu2d.hpp>
#include <simulation/simulation.hpp>

namespace gui {

/// Panel for configuring simulation parameters
class ConfigPanel {
  public:
    ConfigPanel() = default;

    void set_ui_state(UIState::SharedPtr ui_state) { _ui_state = std::move(ui_state); }

    /// Render the panel. Returns true if any configuration changed.
    bool render(float& dt, simulation::SimulationConfig& sim_config, std::shared_ptr<sensors::Camera2D> camera, std::shared_ptr<sensors::IMU2D> imu);

  private:
    UIState::SharedPtr _ui_state;
};

} // namespace gui
