// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include "imgui.h"
#include "implot.h"
#include <gui/color.hpp>
#include <gui/panels/measurements_panel.hpp>
#include <gui/plot.hpp>
#include <gui/ui_state.hpp>
#include <gui/widgets/text.hpp>
#include <map>

namespace gui {

void MeasurementsPanel::render(const simulation::SimulationResult& sim_result, std::shared_ptr<sensors::Camera2D> camera,
                               std::shared_ptr<sensors::IMU2D> imu) {
    if (!sim_result.is_valid()) {
        widgets::Text("No measurements available. Draw a trajectory first.");
        return;
    }

    size_t num_steps = sim_result.num_steps();

    // Prepare time index axis
    std::vector<float> time_axis(num_steps);
    for (size_t i = 0; i < num_steps; ++i) {
        time_axis[i] = static_cast<float>(i);
    }

    // IMU Accelerometer plot
    if (ImPlot::BeginPlot("IMU Accelerometer", ImVec2(-1, 200))) {
        ImPlot::SetupAxes("Time Index", "Acceleration (m/s²)");

        // Extract accelerometer data as Vector2f
        std::vector<Eigen::Vector2f> gt_acc(num_steps), meas_acc(num_steps);
        for (size_t i = 0; i < num_steps; ++i) {
            gt_acc[i] = sim_result.gt_imu[i].acc;
            meas_acc[i] = sim_result.imu_measurements[i].acc;
        }

        // Plot measurement covariance (constant std for all time steps)
        std::vector<Eigen::Vector2f> acc_std(num_steps, imu->acc_noise_std());
        plot_covariance("Acc", time_axis, meas_acc, acc_std);

        // Plot ground truth and measurements
        plot_vector("Acc GT", time_axis, gt_acc, Color::FadedPalette(), 1.0f);
        plot_vector("Acc", time_axis, meas_acc, Color::DefaultPalette(), 2.0f);

        _ui_state->handle_time_selector(num_steps);
        ImPlot::EndPlot();
    }

    // IMU Gyroscope plot
    if (ImPlot::BeginPlot("IMU Gyroscope", ImVec2(-1, 200))) {
        ImPlot::SetupAxes("Time Index", "Angular velocity (rad/s)");

        // Extract data as 1D vectors
        std::vector<Eigen::Vector<float, 1>> gyr_gt(num_steps), gyr_meas(num_steps);
        for (size_t i = 0; i < num_steps; ++i) {
            gyr_gt[i](0) = sim_result.gt_imu[i].gyr;
            gyr_meas[i](0) = sim_result.imu_measurements[i].gyr;
        }

        // Plot measurement covariance (constant std for all time steps)
        std::vector<Eigen::Vector<float, 1>> gyr_std(num_steps);
        for (size_t i = 0; i < num_steps; ++i)
            gyr_std[i](0) = imu->gyr_noise_std();
        plot_covariance("Gyr", time_axis, gyr_meas, gyr_std, Color::DefaultPalette());

        // Plot ground truth and measurements
        plot_vector("Gyr GT", time_axis, gyr_gt, Color::FadedPalette(), 1.0f);
        plot_vector("Gyr", time_axis, gyr_meas, Color::DefaultPalette(), 2.0f);

        _ui_state->handle_time_selector(num_steps);
        ImPlot::EndPlot();
    }

    // Camera measurements plot (Time Index on X axis, Image u on Y axis)
    if (ImPlot::BeginPlot("Camera Observations", ImVec2(-1, 300))) {
        ImPlot::SetupAxes("Time Index", "Image u (px)");
        ImPlot::SetupAxisLimits(ImAxis_X1, 0, static_cast<double>(num_steps));
        ImPlot::SetupAxisLimits(ImAxis_Y1, 0, camera->width());

        // For each landmark, collect observations across time
        // Store as (time_index, u) pairs, then split into consecutive segments
        std::map<size_t, std::vector<std::pair<int, float>>> gt_tracks;
        std::map<size_t, std::vector<std::pair<int, float>>> meas_tracks;

        for (size_t t = 0; t < num_steps; ++t) {
            int time_idx = static_cast<int>(t);
            for (const auto& obs : sim_result.gt_cam[t]) {
                gt_tracks[obs.landmark_id].emplace_back(time_idx, obs.u);
            }
            for (const auto& obs : sim_result.cam_measurements[t]) {
                meas_tracks[obs.landmark_id].emplace_back(time_idx, obs.u);
            }
        }

        // Helper to split track into consecutive segments
        auto split_into_segments = [](const std::vector<std::pair<int, float>>& track) {
            std::vector<std::vector<std::pair<int, float>>> segments;
            if (track.empty())
                return segments;

            std::vector<std::pair<int, float>> current_segment;
            current_segment.push_back(track[0]);

            for (size_t i = 1; i < track.size(); ++i) {
                if (track[i].first == track[i - 1].first + 1) {
                    // Consecutive, add to current segment
                    current_segment.push_back(track[i]);
                } else {
                    // Gap detected, start new segment
                    if (!current_segment.empty()) {
                        segments.push_back(current_segment);
                    }
                    current_segment.clear();
                    current_segment.push_back(track[i]);
                }
            }
            if (!current_segment.empty()) {
                segments.push_back(current_segment);
            }
            return segments;
        };

        // Draw tracks for each landmark
        for (const auto& [lm_id, track] : meas_tracks) {
            if (track.empty())
                continue;

            Color lm_color = Color::Random(lm_id);
            Color gt_lm_color = Color(0.25f * lm_color.r() + 0.75f, 0.25f * lm_color.g() + 0.75f, 0.25f * lm_color.b() + 0.75f);
            std::string label = "LM " + std::to_string(lm_id);

            // Get ground truth track for this landmark
            const auto& gt_track = gt_tracks[lm_id];
            auto gt_segments = split_into_segments(gt_track);
            auto meas_segments = split_into_segments(track);

            // Draw ground truth segments (faded lines)
            for (size_t seg_idx = 0; seg_idx < gt_segments.size(); ++seg_idx) {
                const auto& seg = gt_segments[seg_idx];
                if (seg.size() >= 2) {
                    std::vector<float> seg_t, seg_u;
                    for (const auto& [t, u] : seg) {
                        seg_t.push_back(static_cast<float>(t));
                        seg_u.push_back(u);
                    }
                    ImPlot::SetNextLineStyle(ImVec4(gt_lm_color), 1.0f);
                    ImPlot::PlotLine(label.c_str(), seg_t.data(), seg_u.data(), static_cast<int>(seg_t.size()));
                }
            }

            // Draw ground truth scatter points
            if (!gt_track.empty()) {
                std::vector<float> gt_t, gt_u;
                for (const auto& [t, u] : gt_track) {
                    gt_t.push_back(static_cast<float>(t));
                    gt_u.push_back(u);
                }
                ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 3.0f, ImVec4(gt_lm_color), IMPLOT_AUTO, ImVec4(gt_lm_color));
                ImPlot::PlotScatter(label.c_str(), gt_t.data(), gt_u.data(), static_cast<int>(gt_t.size()));
            }

            // Draw measurement segments (lines)
            for (const auto& seg : meas_segments) {
                if (seg.size() >= 2) {
                    std::vector<float> seg_t, seg_u;
                    for (const auto& [t, u] : seg) {
                        seg_t.push_back(static_cast<float>(t));
                        seg_u.push_back(u);
                    }
                    ImPlot::SetNextLineStyle(ImVec4(lm_color), 2.0f);
                    ImPlot::PlotLine(label.c_str(), seg_t.data(), seg_u.data(), static_cast<int>(seg_t.size()));
                }
            }

            // Draw measurement scatter points
            if (!track.empty()) {
                std::vector<float> meas_t, meas_u;
                for (const auto& [t, u] : track) {
                    meas_t.push_back(static_cast<float>(t));
                    meas_u.push_back(u);
                }
                ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 4.0f, ImVec4(lm_color), IMPLOT_AUTO, ImVec4(lm_color));
                ImPlot::PlotScatter(label.c_str(), meas_t.data(), meas_u.data(), static_cast<int>(meas_t.size()));
            }
        }

        _ui_state->handle_time_selector(num_steps);
        ImPlot::EndPlot();
    }
}

} // namespace gui
