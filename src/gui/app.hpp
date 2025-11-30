// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#pragma once

#include <Eigen/Dense>
#include <core/trajectory2d.hpp>
#include <core/types.hpp>
#include <estimation/imu_integrator.hpp>
#include <gui/panels/config_panel.hpp>
#include <gui/panels/error_metrics_panel.hpp>
#include <gui/panels/measurements_panel.hpp>
#include <gui/panels/perception_output_panel.hpp>
#include <gui/panels/world_editor_panel.hpp>
#include <gui/ui_state.hpp>
#include <memory>
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
    /// Setup initial docking layout
    void setup_docking_layout(ImGuiID dockspace_id);

    /// Render the menu bar
    void render_menu_bar();

    /// Render the about window
    void render_about_window();

    /// Simulate the sensor measurements
    void simulate();

    /// Estimate state given sensor measurements
    void estimate();

    /// Load a world preset
    void load_world_preset(world::Preset preset);

    bool _first_render = true;

    //----- Menu state -----//
    bool _show_imgui_demo = false;
    bool _show_implot_demo = false;
    bool _show_about = false;

    //----- Sensor models -----//
    std::shared_ptr<sensors::Camera2D> _camera;
    std::shared_ptr<sensors::IMU2D> _imu;

    //----- Simulation parameters -----//
    float _dt = 1.0f; ///< Time step between poses (in index units)
    simulation::SimulationConfig _sim_config;

    //----- World & ground-truth states -----//
    world::Preset _current_preset = world::Preset::Custom;

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

    //----- Estimation -----//
    estimation::IMUIntegrator _imu_integrator;
    estimation::EstimationResult _estimation_result;

    //----- Shared UI state -----//
    UIState::SharedPtr _ui_state;

    //----- GUI panels -----//
    ConfigPanel _config_panel;
    WorldEditorPanel _world_editor_panel;
    MeasurementsPanel _measurements_panel;
    PerceptionOutputPanel _perception_output_panel;
    ErrorMetricsPanel _error_metrics_panel;
};

} // namespace gui
