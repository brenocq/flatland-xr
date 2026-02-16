// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#pragma once

#include <core/trajectory2d.hpp>
#include <estimation/estimator_base.hpp>
#include <gui/ui_state.hpp>
#include <simulation/simulation.hpp>

namespace gui {

/// Panel for displaying perception output (estimated vs ground truth)
class PerceptionOutputPanel {
  public:
    PerceptionOutputPanel() = default;

    void set_ui_state(UIState::SharedPtr ui_state) { _ui_state = std::move(ui_state); }

    /// Render the panel
    void render(const estimation::EstimationResult& est_result, const core::Trajectory2D& gt_trajectory,
                const simulation::SimulationResult& sim_result);

  private:
    UIState::SharedPtr _ui_state;
};

} // namespace gui
