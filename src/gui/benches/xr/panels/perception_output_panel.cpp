// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include "imgui.h"
#include "implot.h"
#include <gui/color.hpp>
#include <gui/benches/xr/panels/perception_output_panel.hpp>
#include <gui/plot.hpp>
#include <gui/ui_state.hpp>
#include <gui/widgets/text.hpp>

namespace gui {

void PerceptionOutputPanel::render(const estimation::EstimationResult& est_result, const core::Trajectory2D& gt_trajectory,
                                   const simulation::SimulationResult& sim_result) {
    if (!est_result.is_valid() || !sim_result.is_valid()) {
        widgets::Text("No estimation data available.");
        return;
    }

    size_t num_poses = est_result.num_steps();

    // Prepare time index axis
    std::vector<float> time_axis(num_poses);
    for (size_t i = 0; i < num_poses; i++) {
        time_axis[i] = static_cast<float>(i);
    }

    // Extract poses and velocities from estimation result
    std::vector<Eigen::Vector3f> est_poses = est_result.get_poses();
    std::vector<Eigen::Vector2f> est_vel = est_result.get_velocities();
    std::vector<Eigen::Vector3f> pose_std = est_result.get_pose_std();
    std::vector<Eigen::Vector2f> vel_std = est_result.get_velocity_std();

    // 2D trajectory plot
    if (ImPlot::BeginPlot("Trajectory", ImVec2(-1, 300), ImPlotFlags_Equal)) {
        if (gt_trajectory.is_valid()) {
            plot_2d_trajectory("Ground-truth", gt_trajectory, Color::CatGreen());
        }
        std::vector<Eigen::Vector2f> est_positions;
        est_positions.reserve(num_poses);
        for (const auto& pose : est_poses) {
            est_positions.emplace_back(pose.head<2>());
        }
        plot_2d_path("Estimated", est_positions, Color::CatRed());

        _ui_state->handle_pose_selection(sim_result.gt_poses);
        _ui_state->handle_pose_selection(est_poses);

        ImPlot::EndPlot();
    }

    // Position X and Y plots
    if (ImPlot::BeginPlot("Position", ImVec2(-1, 200))) {
        ImPlot::SetupAxes("Time Index", "Position (m)");

        // Extract position data as Vector2f
        std::vector<Eigen::Vector2f> gt_pos(num_poses), est_pos(num_poses), pos_std_2d(num_poses);
        for (size_t i = 0; i < num_poses; i++) {
            gt_pos[i] = sim_result.gt_poses[i].head<2>();
            est_pos[i] = est_poses[i].head<2>();
            pos_std_2d[i] = pose_std[i].head<2>(); // Extract X and Y std
        }

        // Plot estimation covariance
        plot_covariance("Est", time_axis, est_pos, pos_std_2d);

        // Plot ground truth and estimated
        plot_vector("GT", time_axis, gt_pos, Color::FadedPalette(), 1.0f);
        plot_vector("Est", time_axis, est_pos, Color::DefaultPalette(), 2.0f);

        _ui_state->handle_time_selector(num_poses);
        ImPlot::EndPlot();
    }

    // Orientation plot
    if (ImPlot::BeginPlot("Orientation", ImVec2(-1, 200))) {
        ImPlot::SetupAxes("Time Index", "Orientation (rad)");

        // Extract orientation data as 1D vectors
        std::vector<Eigen::Vector<float, 1>> gt_theta(num_poses), est_theta(num_poses), theta_std(num_poses);
        for (size_t i = 0; i < num_poses; i++) {
            gt_theta[i](0) = sim_result.gt_poses[i].z();
            est_theta[i](0) = est_poses[i].z();
            theta_std[i](0) = pose_std[i].z(); // Extract theta std
        }

        // Plot estimation covariance
        plot_covariance("Est Theta", time_axis, est_theta, theta_std);

        // Plot ground truth and estimated
        plot_vector("GT Theta", time_axis, gt_theta, Color::FadedPalette(), 1.0f);
        plot_vector("Est Theta", time_axis, est_theta, Color::DefaultPalette(), 2.0f);

        _ui_state->handle_time_selector(num_poses);
        ImPlot::EndPlot();
    }

    // Velocity plot
    if (ImPlot::BeginPlot("Velocity", ImVec2(-1, 200))) {
        ImPlot::SetupAxes("Time Index", "Velocity (m/idx)");

        // Plot estimation covariance
        plot_covariance("Est V", time_axis, est_vel, vel_std);

        // Plot ground truth and estimated
        plot_vector("GT V", time_axis, sim_result.gt_vel, Color::FadedPalette(), 1.0f);
        plot_vector("Est V", time_axis, est_vel, Color::DefaultPalette(), 2.0f);

        _ui_state->handle_time_selector(num_poses);
        ImPlot::EndPlot();
    }
}

} // namespace gui
