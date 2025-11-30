// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#pragma once

#include <core/trajectory2d.hpp>
#include <estimation/estimator_base.hpp>
#include <simulation/simulation.hpp>

namespace gui {

/// Panel for displaying perception output (estimated vs ground truth)
class PerceptionOutputPanel {
  public:
    PerceptionOutputPanel() = default;

    /// Render the panel
    void render(const estimation::EstimationResult& est_result, const core::Trajectory2D& gt_trajectory,
                const simulation::SimulationResult& sim_result);
};

} // namespace gui
