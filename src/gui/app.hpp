// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#pragma once

#include <Eigen/Dense>
#include <core/trajectory2d.hpp>
#include <core/types.hpp>
#include <gui/panels/config_panel.hpp>
#include <gui/panels/error_metrics_panel.hpp>
#include <gui/panels/measurements_panel.hpp>
#include <gui/panels/perception_output_panel.hpp>
#include <gui/panels/world_editor_panel.hpp>
#include <sensors/camera2d.hpp>
#include <sensors/imu2d.hpp>
#include <simulation/simulation.hpp>
#include <vector>
#include <world/world.hpp>

namespace gui {

class App {
  public:
    App();

    /// Startup app internal data structures
    void startup();

    /// Shutdown the app and cleanup internal data structures
    void shutdown();

    /// Update app logic and render frame
    void update();

  private:
    /// Render the app's ImGui window
    void render();

    /// Simulate the sensor measurements
    void simulate();

    /// Estimate state given sensor measurements
    void estimate();

    /// Load a world preset
    void load_world_preset(world::Preset preset);

    bool _first_render = true;

    //----- Sensor models -----//
    sensors::Camera2D _camera;
    sensors::IMU2D _imu;

    //----- Simulation parameters -----//
    float _dt = 1.0f; ///< Time step between poses (in index units)
    simulation::SimulationConfig _sim_config;

    //----- World & ground-truth states -----//
    world::Preset _current_preset = world::Preset::Custom;
    world::Preset _pending_preset = world::Preset::Custom; ///< Preset to load (set by panel)

    // Trajectory
    std::vector<Eigen::Vector3f> _gt_pose_raw; ///< Raw poses from mouse input (x, y, orientation)
    core::Trajectory2D _gt_trajectory;         ///< Smoothed ground truth trajectory

    // Landmarks
    std::vector<Eigen::Vector2f> _landmarks;

    // Walls
    std::vector<core::Wall> _walls;
    std::vector<Eigen::Vector2f> _wall_raw_points; ///< Raw points while drawing a wall

    //----- Simulation result -----//
    simulation::SimulationResult _sim_result;

    //----- Estimated data -----//
    std::vector<Eigen::Vector3f> _est_poses; ///< Estimated poses (x, y, theta)
    std::vector<Eigen::Vector2f> _est_vel;   ///< Estimated velocities (vx, vy)

    //----- GUI panels -----//
    ConfigPanel _config_panel;
    WorldEditorPanel _world_editor_panel;
    MeasurementsPanel _measurements_panel;
    PerceptionOutputPanel _perception_output_panel;
    ErrorMetricsPanel _error_metrics_panel;
};

} // namespace gui
