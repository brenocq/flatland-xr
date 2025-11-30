// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include "imgui.h"
#include "implot.h"

#include <cmath>
#include <gui/app.hpp>

namespace gui {

App::App() {}

void App::startup() {
    _camera.set_intrinsics(100, 60.0f * M_PI / 180.0f);
    _camera.set_noise_std(1.0f);
    _imu.set_acc_bias(Eigen::Vector2f(0.0f, 0.0f));
    _imu.set_gyr_bias(0.0f);
    _imu.set_acc_noise_std(Eigen::Vector2f(0.01f, 0.01f));
    _imu.set_gyr_noise_std(0.001f);
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
    // Docking
    ImGuiID dockId = ImGui::GetID("##DockSpace");
    ImGui::DockSpaceOverViewport(dockId, ImGui::GetMainViewport());
    ImGui::SetNextWindowDockID(dockId, ImGuiCond_Appearing);
    if (ImGui::Begin("Flatland XR")) {
        render();
    }
    ImGui::End();

    // ImGui::ShowDemoWindow();
    // ImPlot::ShowDemoWindow();
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
    _perception_output_panel.render(_est_poses, _est_vel, _gt_trajectory, _sim_result);
    _error_metrics_panel.render();
}

void App::simulate() {
    // Clear previous data
    _sim_result.clear();
    _est_poses.clear();
    _est_vel.clear();

    if (!_gt_trajectory.is_valid())
        return;

    // Run simulation
    _sim_result = simulation::run(_gt_trajectory, _landmarks, _walls, _camera, _imu, _sim_config);
}

void App::estimate() {
    _est_poses.clear();
    _est_vel.clear();

    if (!_sim_result.is_valid())
        return;

    size_t num_poses = _sim_result.num_steps();
    _est_poses.reserve(num_poses);
    _est_vel.reserve(num_poses);

    // Initialize with ground truth initial state
    _est_poses.push_back(_sim_result.gt_poses[0]);
    _est_vel.push_back(_sim_result.gt_vel[0]);

    // Integrate IMU measurements (dt=1 since trajectory is parameterized by index)
    const float dt = 1.0f;

    for (size_t i = 1; i < num_poses; i++) {
        const sensors::IMUMeasurement& imu = _sim_result.imu_measurements[i - 1];
        Eigen::Vector3f prev_pose = _est_poses[i - 1];
        Eigen::Vector2f prev_vel = _est_vel[i - 1];
        float theta = prev_pose.z();

        // Remove bias from IMU measurements (we know the true bias for now)
        Eigen::Vector2f acc_body = imu.acc - _imu.acc_bias();
        float gyr = imu.gyr - _imu.gyr_bias();

        // Rotate acceleration from body frame to world frame
        float cos_t = std::cos(theta);
        float sin_t = std::sin(theta);
        Eigen::Matrix2f R_bw; // Body to world rotation
        R_bw << cos_t, -sin_t, sin_t, cos_t;
        Eigen::Vector2f acc_world = R_bw * acc_body;

        // Add gravity back (accelerometer measures specific force = acc - gravity)
        // So world_acc = specific_force + gravity
        acc_world += _sim_config.gravity;

        // Integrate orientation: theta_new = theta + omega * dt
        float new_theta = theta + gyr * dt;

        // Integrate velocity: v_new = v + a * dt
        Eigen::Vector2f new_vel = prev_vel + acc_world * dt;

        // Integrate position: p_new = p + v * dt + 0.5 * a * dt^2
        Eigen::Vector2f new_pos = prev_pose.head<2>() + prev_vel * dt + 0.5f * acc_world * dt * dt;

        _est_poses.push_back(Eigen::Vector3f(new_pos.x(), new_pos.y(), new_theta));
        _est_vel.push_back(new_vel);
    }
}

} // namespace gui
