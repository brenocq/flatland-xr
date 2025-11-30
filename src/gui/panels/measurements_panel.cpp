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

void MeasurementsPanel::render(const simulation::SimulationResult& sim_result, const sensors::Camera2D& camera) {
    if (!sim_result.is_valid()) {
        widgets::Text("No measurements available. Draw a trajectory first.");
        return;
    }

    size_t num_steps = sim_result.num_steps();

    // Prepare time index axis
    std::vector<float> time_axis(num_steps);
    for (size_t i = 0; i < num_steps; i++) {
        time_axis[i] = static_cast<float>(i);
    }

    // IMU Accelerometer plot
    if (ImPlot::BeginPlot("IMU Accelerometer", ImVec2(-1, 200))) {
        ImPlot::SetupAxes("Time Index", "Acceleration (m/s²)");

        // Extract data
        std::vector<float> gt_acc_x(num_steps), gt_acc_y(num_steps);
        std::vector<float> meas_acc_x(num_steps), meas_acc_y(num_steps);
        for (size_t i = 0; i < num_steps; i++) {
            gt_acc_x[i] = sim_result.gt_imu[i].acc.x();
            gt_acc_y[i] = sim_result.gt_imu[i].acc.y();
            meas_acc_x[i] = sim_result.imu_measurements[i].acc.x();
            meas_acc_y[i] = sim_result.imu_measurements[i].acc.y();
        }

        // Colors: measurement colors and faded ground truth
        Color color_x = Color::CatRed();
        Color color_y = Color::CatBlue();
        Color gt_color_x = Color(0.25f * color_x.r() + 0.75f, 0.25f * color_x.g() + 0.75f, 0.25f * color_x.b() + 0.75f);
        Color gt_color_y = Color(0.25f * color_y.r() + 0.75f, 0.25f * color_y.g() + 0.75f, 0.25f * color_y.b() + 0.75f);

        // Plot ground truth (faded)
        ImPlot::SetNextLineStyle(ImVec4(gt_color_x), 1.0f);
        ImPlot::PlotLine("GT Acc X", time_axis.data(), gt_acc_x.data(), static_cast<int>(num_steps));
        ImPlot::SetNextLineStyle(ImVec4(gt_color_y), 1.0f);
        ImPlot::PlotLine("GT Acc Y", time_axis.data(), gt_acc_y.data(), static_cast<int>(num_steps));

        // Plot measurements
        ImPlot::SetNextLineStyle(ImVec4(color_x), 2.0f);
        ImPlot::PlotLine("Acc X", time_axis.data(), meas_acc_x.data(), static_cast<int>(num_steps));
        ImPlot::SetNextLineStyle(ImVec4(color_y), 2.0f);
        ImPlot::PlotLine("Acc Y", time_axis.data(), meas_acc_y.data(), static_cast<int>(num_steps));

        _ui_state->handle_time_selector(num_steps);
        ImPlot::EndPlot();
    }

    // IMU Gyroscope plot
    if (ImPlot::BeginPlot("IMU Gyroscope", ImVec2(-1, 200))) {
        ImPlot::SetupAxes("Time Index", "Angular velocity (rad/s)");

        // Extract data
        std::vector<float> gt_gyr(num_steps), meas_gyr(num_steps);
        for (size_t i = 0; i < num_steps; i++) {
            gt_gyr[i] = sim_result.gt_imu[i].gyr;
            meas_gyr[i] = sim_result.imu_measurements[i].gyr;
        }

        // Colors
        Color color_gyr = Color::CatGreen();
        Color gt_color_gyr = Color(0.25f * color_gyr.r() + 0.75f, 0.25f * color_gyr.g() + 0.75f, 0.25f * color_gyr.b() + 0.75f);

        // Plot ground truth (faded)
        ImPlot::SetNextLineStyle(ImVec4(gt_color_gyr), 1.0f);
        ImPlot::PlotLine("GT Gyro", time_axis.data(), gt_gyr.data(), static_cast<int>(num_steps));

        // Plot measurements
        ImPlot::SetNextLineStyle(ImVec4(color_gyr), 2.0f);
        ImPlot::PlotLine("Gyro", time_axis.data(), meas_gyr.data(), static_cast<int>(num_steps));

        _ui_state->handle_time_selector(num_steps);
        ImPlot::EndPlot();
    }

    // Camera measurements plot (Time Index on X axis, Image u on Y axis)
    if (ImPlot::BeginPlot("Camera Observations", ImVec2(-1, 300))) {
        ImPlot::SetupAxes("Time Index", "Image u (px)");
        ImPlot::SetupAxisLimits(ImAxis_X1, 0, static_cast<double>(num_steps));
        ImPlot::SetupAxisLimits(ImAxis_Y1, 0, camera.width());

        // For each landmark, collect observations across time
        // Store as (time_index, u) pairs, then split into consecutive segments
        std::map<size_t, std::vector<std::pair<int, float>>> gt_tracks;
        std::map<size_t, std::vector<std::pair<int, float>>> meas_tracks;

        for (size_t t = 0; t < num_steps; t++) {
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

            for (size_t i = 1; i < track.size(); i++) {
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
            for (size_t seg_idx = 0; seg_idx < gt_segments.size(); seg_idx++) {
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
