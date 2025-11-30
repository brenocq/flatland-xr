// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include "imgui.h"
#include "implot.h"
#include <gui/color.hpp>
#include <gui/panels/perception_output_panel.hpp>
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

        std::vector<float> gt_x(num_poses), gt_y(num_poses);
        std::vector<float> est_x(num_poses), est_y(num_poses);
        for (size_t i = 0; i < num_poses; i++) {
            gt_x[i] = sim_result.gt_poses[i].x();
            gt_y[i] = sim_result.gt_poses[i].y();
            est_x[i] = est_poses[i].x();
            est_y[i] = est_poses[i].y();
        }

        // Ground truth (faded)
        Color gt_color_x = Color(0.25f * 1.0f + 0.75f, 0.25f * 0.0f + 0.75f, 0.25f * 0.0f + 0.75f);
        Color gt_color_y = Color(0.25f * 0.0f + 0.75f, 0.25f * 0.0f + 0.75f, 0.25f * 1.0f + 0.75f);
        ImPlot::SetNextLineStyle(ImVec4(gt_color_x), 1.0f);
        ImPlot::PlotLine("GT X", time_axis.data(), gt_x.data(), static_cast<int>(num_poses));
        ImPlot::SetNextLineStyle(ImVec4(gt_color_y), 1.0f);
        ImPlot::PlotLine("GT Y", time_axis.data(), gt_y.data(), static_cast<int>(num_poses));

        // Estimated
        ImPlot::SetNextLineStyle(ImVec4(Color::CatRed()), 2.0f);
        ImPlot::PlotLine("Est X", time_axis.data(), est_x.data(), static_cast<int>(num_poses));
        ImPlot::SetNextLineStyle(ImVec4(Color::CatBlue()), 2.0f);
        ImPlot::PlotLine("Est Y", time_axis.data(), est_y.data(), static_cast<int>(num_poses));

        _ui_state->handle_time_selector(num_poses);
        ImPlot::EndPlot();
    }

    // Orientation plot
    if (ImPlot::BeginPlot("Orientation", ImVec2(-1, 200))) {
        ImPlot::SetupAxes("Time Index", "Orientation (rad)");

        std::vector<float> gt_theta(num_poses), est_theta(num_poses);
        for (size_t i = 0; i < num_poses; i++) {
            gt_theta[i] = sim_result.gt_poses[i].z();
            est_theta[i] = est_poses[i].z();
        }

        Color gt_color = Color(0.25f * 0.0f + 0.75f, 0.25f * 0.5f + 0.75f, 0.25f * 0.0f + 0.75f);
        ImPlot::SetNextLineStyle(ImVec4(gt_color), 1.0f);
        ImPlot::PlotLine("GT Theta", time_axis.data(), gt_theta.data(), static_cast<int>(num_poses));

        ImPlot::SetNextLineStyle(ImVec4(Color::CatGreen()), 2.0f);
        ImPlot::PlotLine("Est Theta", time_axis.data(), est_theta.data(), static_cast<int>(num_poses));

        _ui_state->handle_time_selector(num_poses);
        ImPlot::EndPlot();
    }

    // Velocity plot
    if (ImPlot::BeginPlot("Velocity", ImVec2(-1, 200))) {
        ImPlot::SetupAxes("Time Index", "Velocity (m/idx)");

        // Plot ground truth and estimated
        plot_vector("GT V", time_axis, sim_result.gt_vel, Color::FadedPalette(), 1.0f);
        plot_vector("Est V", time_axis, est_vel, Color::DefaultPalette(), 2.0f);

        _ui_state->handle_time_selector(num_poses);
        ImPlot::EndPlot();
    }
}

} // namespace gui
