// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#pragma once

#include <Eigen/Dense>
#include <camera2d.hpp>
#include <trajectory2d.hpp>
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

    /// Render world editor for drawing trajectory and placing landmarks
    void render_world_editor();

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

    //----- World Preset -----//
    WorldPreset _current_preset = WorldPreset::Custom;

    //----- Simulation Configuration -----//
    int _num_steps = 100;
    int _num_landmarks = 10;

    // Camera parameters
    float _cam_noise_std = 1.0f;
    Camera2D _camera; ///< Camera model for projection

    // IMU parameters
    float _acc_noise_std = 0.01f;
    float _gyr_noise_std = 0.1f;

    //----- Simulated data -----//
    std::vector<Eigen::Vector3f> _gt_pose_raw; ///< Raw poses from mouse input (x, y, orientation)
    Trajectory2D _gt_trajectory;               ///< Smoothed ground truth trajectory
    std::vector<Eigen::Vector2f> _landmarks;

    // Wall data
    struct Wall {
        std::vector<Eigen::Vector2f> points; ///< Line segment points forming the wall
    };
    std::vector<Wall> _walls;
    std::vector<Eigen::Vector2f> _wall_raw_points; ///< Raw points while drawing a wall

    // For simplicity, all measurements are available at each time step
    // IMU measurements
    std::vector<Eigen::Vector2f> _imu_acc;
    std::vector<float> _imu_gyr;
    // Camera measurements
    struct Observation {
        Eigen::Vector2f uv;
        size_t landmark_id;
    };
    struct Frame {
        std::vector<Observation> observations;
    };
    std::vector<Frame> _cam_frames;

    //----- Estimated data -----//
    std::vector<Eigen::Vector2f> _est_pos;
};
