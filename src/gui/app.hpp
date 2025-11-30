// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#pragma once

#include <Eigen/Dense>
#include <core/trajectory2d.hpp>
#include <core/types.hpp>
#include <sensors/camera2d.hpp>
#include <sensors/imu2d.hpp>
#include <vector>

/// World preset identifiers
enum class WorldPreset { Custom = 0, ASquaresHouse, VisitFromSphere, HallOfCouncil, COUNT };

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

    /// Render config header to setup simulation parameters. Returns true if the config changed.
    bool render_config();

    /// Render world editor for drawing trajectory and placing landmarks. Returns true if world changed.
    bool render_world_editor();

    /// Render sensor measurements
    void render_measurements();

    /// Render perception output
    void render_perception_output();

    /// Render error metrics
    void render_error_metrics();

    /// Smooth raw poses and build trajectory
    void build_trajectory_from_raw_poses();

    /// Simulate the sensor measurements
    void simulate();

    /// Estimate state given sensor measurements
    void estimate();

    /// Smooth raw wall points and build simplified wall
    void build_wall_from_raw_points();

    /// Check if a ray from camera to landmark is blocked by any wall
    bool is_landmark_occluded_by_walls(const Eigen::Vector2f& camera_pos, const Eigen::Vector2f& landmark) const;

    /// Filter landmarks that are visible (not occluded by walls)
    std::vector<Eigen::Vector2f> filter_visible_landmarks(const Eigen::Vector2f& camera_pos) const;

    /// Load a world preset
    void load_world_preset(WorldPreset preset);

    bool _first_render = true;

    //----- Sensor models -----//
    sensors::Camera2D _camera;
    sensors::IMU2D _imu;

    //----- Simulation parameters -----//
    float _dt = 1.0f;                                   ///< Time step between poses (in index units)
    Eigen::Vector2f _gravity = Eigen::Vector2f::Zero(); ///< World gravity vector (zero for 2D planar)

    //----- World & ground-truth states -----//
    WorldPreset _current_preset = WorldPreset::Custom;

    // Trajectory
    std::vector<Eigen::Vector3f> _gt_pose_raw; ///< Raw poses from mouse input (x, y, orientation)
    core::Trajectory2D _gt_trajectory;         ///< Smoothed ground truth trajectory

    // Ground truth states at each time step (sampled from trajectory)
    std::vector<Eigen::Vector3f> _gt_poses; ///< Ground truth poses (x, y, theta)
    std::vector<Eigen::Vector2f> _gt_vel;   ///< Ground truth velocities (vx, vy)
    std::vector<Eigen::Vector2f> _gt_acc;   ///< Ground truth accelerations (ax, ay)
    std::vector<float> _gt_omega;           ///< Ground truth angular velocities

    // Landmarks
    std::vector<Eigen::Vector2f> _landmarks;

    // Walls
    std::vector<core::Wall> _walls;
    std::vector<Eigen::Vector2f> _wall_raw_points; ///< Raw points while drawing a wall

    //----- Simulated measurements -----//
    std::vector<sensors::IMUMeasurement> _gt_imu; ///< Ground truth IMU (no noise, with bias)
    std::vector<sensors::IMUMeasurement> _imu_measurements;

    std::vector<std::vector<sensors::LandmarkObservation>> _gt_cam; ///< Ground truth camera (no noise)
    std::vector<std::vector<sensors::LandmarkObservation>> _cam_measurements;

    //----- Estimated data -----//
    std::vector<Eigen::Vector3f> _est_poses; ///< Estimated poses (x, y, theta)
    std::vector<Eigen::Vector2f> _est_vel;   ///< Estimated velocities (vx, vy)
};
