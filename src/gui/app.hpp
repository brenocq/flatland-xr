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

    int _cam_width = 100; ///< Camera width in pixels
    float _cam_fov = 45.0f; ///< Camera FOV in degrees

    float _acc_noise_std = 0.01f;
    float _gyr_noise_std = 0.1f;
    float _cam_noise_std = 1.0f;

    //----- Simulated data -----//
    std::vector<Eigen::Vector2f> _gt_pos;

    //----- Estimated data -----//
    std::vector<Eigen::Vector2f> _est_pos;
};
