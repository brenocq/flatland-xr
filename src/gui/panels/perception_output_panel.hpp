// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#pragma once

#include <Eigen/Dense>
#include <core/trajectory2d.hpp>
#include <simulation/simulation.hpp>
#include <vector>

namespace gui {

/// Panel for displaying perception output (estimated vs ground truth)
class PerceptionOutputPanel {
  public:
    PerceptionOutputPanel() = default;

    /// Render the panel
    void render(const std::vector<Eigen::Vector3f>& est_poses, const std::vector<Eigen::Vector2f>& est_vel, const core::Trajectory2D& gt_trajectory,
                const simulation::SimulationResult& sim_result);
};

} // namespace gui
