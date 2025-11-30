// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#pragma once

#include <sensors/camera2d.hpp>
#include <sensors/imu2d.hpp>
#include <simulation/simulation.hpp>

namespace gui {

/// Panel for configuring simulation parameters
class ConfigPanel {
  public:
    ConfigPanel() = default;

    /// Render the panel. Returns true if any configuration changed.
    bool render(float& dt, simulation::SimulationConfig& sim_config, sensors::Camera2D& camera, sensors::IMU2D& imu);
};

} // namespace gui
