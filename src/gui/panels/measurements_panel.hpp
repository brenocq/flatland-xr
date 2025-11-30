// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#pragma once

#include <sensors/camera2d.hpp>
#include <simulation/simulation.hpp>

namespace gui {

/// Panel for displaying sensor measurements
class MeasurementsPanel {
  public:
    MeasurementsPanel() = default;

    /// Render the panel
    void render(const simulation::SimulationResult& sim_result, const sensors::Camera2D& camera);
};

} // namespace gui
