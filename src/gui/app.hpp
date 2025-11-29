// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#pragma once

#include <Eigen/Dense>
#include <vector>

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

    /// Simulate the sensor measurements
    void simulate();

    /// Estimate state given sensor measurements
    void estimate();

    bool _first_render = true;

    //----- Simulation Configuration -----//
    int _num_steps = 100;
    int _num_landmarks = 10;

    // Camera parameters
    int _cam_width = 100;   ///< Camera width in pixels
    float _cam_fov = 45.0f; ///< Camera FOV in degrees
    float _cam_noise_std = 1.0f;

    // IMU parameters
    float _acc_noise_std = 0.01f;
    float _gyr_noise_std = 0.1f;

    //----- Simulated data -----//
    std::vector<Eigen::Vector2f> _gt_pos;
    std::vector<Eigen::Vector2f> _landmarks;

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
