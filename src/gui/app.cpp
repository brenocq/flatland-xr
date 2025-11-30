// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include "imgui.h"
#include "implot.h"

#include <core/math.hpp>
#include <gui/app.hpp>

namespace gui {

App::App() : _ui_state(std::make_shared<UIState>()) {
    _config_panel.set_ui_state(_ui_state);
    _world_editor_panel.set_ui_state(_ui_state);
    _measurements_panel.set_ui_state(_ui_state);
    _perception_output_panel.set_ui_state(_ui_state);
    _error_metrics_panel.set_ui_state(_ui_state);
}

void App::startup() {
    _camera.set_intrinsics(100, 60.0f * core::DEG_TO_RAD);
    _camera.set_noise_std(1.0f);
    _imu.set_acc_bias(Eigen::Vector2f(0.0f, 0.0f));
    _imu.set_gyr_bias(0.0f);
    _imu.set_acc_noise_std(Eigen::Vector2f(0.01f, 0.01f));
    _imu.set_gyr_noise_std(0.001f);

    // Setup IMU integrator
    _imu_integrator.set_imu_model(&_imu);
    _imu_integrator.set_gravity(_sim_config.gravity);
    _imu_integrator.set_process_noise(0.01f, 0.001f);

    load_world_preset(world::Preset::ASquaresHouse);
}

void App::shutdown() {}

void App::load_world_preset(world::Preset preset) {
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

void App::update() {
    // Menu bar
    render_menu_bar();

    // Docking
    ImGuiID dockId = ImGui::GetID("##DockSpace");
    ImGui::DockSpaceOverViewport(dockId, ImGui::GetMainViewport());
    ImGui::SetNextWindowDockID(dockId, ImGuiCond_Appearing);
    if (ImGui::Begin("Flatland XR")) {
        render();
    }
    ImGui::End();

    // Demo windows
    if (_show_imgui_demo) {
        ImGui::ShowDemoWindow(&_show_imgui_demo);
    }
    if (_show_implot_demo) {
        ImPlot::ShowDemoWindow(&_show_implot_demo);
    }
    if (_show_about) {
        render_about_window();
    }
}

void App::render() {
    bool should_simulate = _first_render;
    should_simulate |= _config_panel.render(_dt, _sim_config, _camera, _imu);

    // Handle world preset changes from the panel
    world::Preset prev_preset = _current_preset;
    bool world_changed = _world_editor_panel.render(_current_preset, _gt_pose_raw, _gt_trajectory, _landmarks, _walls, _wall_raw_points, _camera);

    // If preset changed via combo box, load the new preset
    if (_current_preset != prev_preset && _current_preset != world::Preset::Custom) {
        load_world_preset(_current_preset);
        world_changed = true;
    }

    should_simulate |= world_changed;
    if (should_simulate) {
        _first_render = false;
        simulate();
        estimate();
    }
    _measurements_panel.render(_sim_result, _camera);
    _perception_output_panel.render(_estimation_result, _gt_trajectory, _sim_result);
    _error_metrics_panel.render();
}

void App::simulate() {
    // Clear previous data
    _sim_result.clear();
    _estimation_result.clear();

    if (!_gt_trajectory.is_valid())
        return;

    // Run simulation
    _sim_result = simulation::run(_gt_trajectory, _landmarks, _walls, _camera, _imu, _sim_config);
}

void App::estimate() {
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

void App::render_menu_bar() {
    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("Windows")) {
            ImGui::MenuItem("ImGui Demo", nullptr, &_show_imgui_demo);
            ImGui::MenuItem("ImPlot Demo", nullptr, &_show_implot_demo);
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Help")) {
            ImGui::MenuItem("About Flatland XR", nullptr, &_show_about);
            ImGui::EndMenu();
        }
        ImGui::EndMainMenuBar();
    }
}

void App::render_about_window() {
    if (!ImGui::Begin("About Flatland XR", &_show_about, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::End();
        return;
    }

    ImGui::Text("Flatland XR v0.1.0");
    ImGui::Separator();

    ImGui::TextLinkOpenURL("Homepage", "https://github.com/brenocq/flatland-xr");
    ImGui::SameLine();
    ImGui::TextLinkOpenURL("Issues", "https://github.com/brenocq/flatland-xr/issues");
    ImGui::SameLine();
    ImGui::TextLinkOpenURL("Releases", "https://github.com/brenocq/flatland-xr/releases");
    ImGui::SameLine();
    ImGui::TextLinkOpenURL("Sponsor", "https://github.com/sponsors/brenocq");

    ImGui::Separator();
    ImGui::Text("(c) 2025 Breno Cunha Queiroz");
    ImGui::Text("Flatland XR is licensed under the MIT License.");
    ImGui::Text("If you enjoy Flatland XR, please consider sponsoring the project.");

    ImGui::End();
}

} // namespace gui
