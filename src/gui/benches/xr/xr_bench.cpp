// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include "imgui.h"
#include "imgui_internal.h"

#include <core/math.hpp>
#include <gui/benches/xr/xr_bench.hpp>
#include <gui/color.hpp>

namespace gui {

XRBench::XRBench()
    : Bench("XR"), _camera(std::make_shared<sensors::Camera2D>()), _imu(std::make_shared<sensors::IMU2D>()), _ui_state(std::make_shared<UIState>()) {
    _config_panel.set_ui_state(_ui_state);
    _world_editor_panel.set_ui_state(_ui_state);
    _measurements_panel.set_ui_state(_ui_state);
    _perception_output_panel.set_ui_state(_ui_state);
    _error_metrics_panel.set_ui_state(_ui_state);

    // Initialize sensor models
    _camera->set_intrinsics(100, 60.0f * core::DEG_TO_RAD);
    _camera->set_noise_std(1.0f);
    _imu->set_acc_bias(Eigen::Vector2f(0.0f, 0.0f));
    _imu->set_gyr_bias(0.0f);
    _imu->set_acc_noise_std(Eigen::Vector2f(0.01f, 0.01f));
    _imu->set_gyr_noise_std(0.001f);

    // Setup IMU integrator
    _imu_integrator.set_imu_model(_imu);
    _imu_integrator.set_gravity(_sim_config.gravity);
    _imu_integrator.set_process_noise(0.01f, 0.001f);

    // Load default world
    load_world_preset(world::Preset::ASquaresHouse);
}

void XRBench::load_world_preset(world::Preset preset) {
    _current_preset = preset;
    _gt_pose_raw.clear();
    _gt_trajectory = core::Trajectory2D();
    _landmarks.clear();
    _walls.clear();
    _wall_raw_points.clear();

    if (preset == world::Preset::Custom) {
        return;
    }

    // Load preset data
    world::World world_data = world::load_preset(preset);
    _landmarks = world_data.landmarks;
    _walls = world_data.walls;

    // Generate dense trajectory from keypoints
    if (world_data.trajectory_keypoints.size() >= 2) {
        auto dense_poses = world::interpolate_trajectory(world_data.trajectory_keypoints, 60);
        _gt_trajectory.build(dense_poses);
    }
}

void XRBench::render() {
    // Create a window for the XR bench
    if (!ImGui::Begin("XR Bench", nullptr, ImGuiWindowFlags_None)) {
        ImGui::End();
        return;
    }

    bool should_simulate = _first_render;

    // Left side: Tabs with main content
    {
        ImGui::BeginChild("left pane", ImVec2(ImGui::GetContentRegionAvail().x * 0.75f, 0), ImGuiChildFlags_None);

        if (ImGui::BeginTabBar("XRTabs", ImGuiTabBarFlags_None)) {
            if (ImGui::BeginTabItem("World Editor")) {
                world::Preset prev_preset = _current_preset;
                bool world_changed =
                    _world_editor_panel.render(_current_preset, _gt_pose_raw, _gt_trajectory, _landmarks, _walls, _wall_raw_points, _camera);

                // If preset changed via combo box, load the new preset
                if (_current_preset != prev_preset && _current_preset != world::Preset::Custom) {
                    load_world_preset(_current_preset);
                    world_changed = true;
                }

                should_simulate |= world_changed;
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("Sensor Measurements")) {
                _measurements_panel.render(_sim_result, _camera, _imu);
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("Perception Output")) {
                _perception_output_panel.render(_estimation_result, _gt_trajectory, _sim_result);
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("Error Metrics")) {
                _error_metrics_panel.render();
                ImGui::EndTabItem();
            }

            ImGui::EndTabBar();
        }

        ImGui::EndChild();
    }

    ImGui::SameLine();

    // Right side: Config panel
    {
        ImGui::BeginChild("config pane", ImVec2(0, 0), ImGuiChildFlags_Borders);
        ImGui::TextColored(Color::CatSapphire(), "Configuration");
        ImGui::Separator();
        should_simulate |= _config_panel.render(_dt, _sim_config, _camera, _imu);
        ImGui::EndChild();
    }

    if (should_simulate) {
        simulate();
        estimate();
    }

    _first_render = false;

    ImGui::End();
}

void XRBench::simulate() {
    // Clear previous data
    _sim_result.clear();
    _estimation_result.clear();

    if (!_gt_trajectory.is_valid())
        return;

    // Run simulation
    _sim_result = simulation::run(_gt_trajectory, _landmarks, _walls, _camera, _imu, _sim_config);
}

void XRBench::estimate() {
    _estimation_result.clear();

    if (!_sim_result.is_valid())
        return;

    // Update IMU integrator settings
    _imu_integrator.set_gravity(_sim_config.gravity);

    // Initialize with ground truth initial state
    _imu_integrator.reset();
    _imu_integrator.initialize(_sim_result.gt_poses[0], _sim_result.gt_vel[0]);

    // Integrate IMU measurements (dt=1 since trajectory is parameterized by index)
    const float dt = 1.0f;

    for (size_t i = 0; i < _sim_result.num_steps() - 1; i++) {
        _imu_integrator.process_imu(_sim_result.imu_measurements[i], dt);
    }

    _estimation_result = _imu_integrator.get_result();
}

} // namespace gui
