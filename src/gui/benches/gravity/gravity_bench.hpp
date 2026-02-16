// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#pragma once

#include <Eigen/Dense>
#include <gui/benches/bench.hpp>
#include <vector>

namespace gui {

/// Preset configurations for gravity simulations
enum class GravityPreset { Custom, TwinStars, SolarSystem, ThreeBody, OrbitingPlanets };

/// A gravitational body with mass, position, and velocity
struct GravityBody {
    double mass;
    Eigen::Vector2d position;
    Eigen::Vector2d velocity;

    GravityBody() : mass(1.0), position(0.0, 0.0), velocity(0.0, 0.0) {}
    GravityBody(double m, const Eigen::Vector2d& pos, const Eigen::Vector2d& vel) : mass(m), position(pos), velocity(vel) {}
};

/// Gravity bench to simulate gravity effects
class GravityBench : public Bench {
  public:
    GravityBench();

    void render() override;

  private:
    /// Render the configuration panel
    void render_config_panel();

    /// Load a preset configuration
    void load_preset(GravityPreset preset);

    /// Compute gravitational potential at a point
    double potential_func(double x, double y);

    /// Simulate one time step
    void simulate_step(double dt);

    /// Reset simulation to initial conditions
    void reset_simulation();

    bool _first_render = true;
    GravityPreset _current_preset = GravityPreset::TwinStars;

    // Bodies state
    std::vector<GravityBody> _bodies;
    std::vector<GravityBody> _initial_bodies; // Store initial conditions for reset

    // Trajectory trails (circular buffer for each body)
    std::vector<std::vector<Eigen::Vector2d>> _trajectories;
    double _trail_duration = 1.0;     // Keep last 5 seconds of trajectory
    size_t _max_trail_points = 10000; // Maximum points per trajectory

    // Simulation parameters
    bool _is_playing = false;
    double _simulation_time = 0.0;
    float _time_step = 0.0001f;           // Time step for integration
    float _playback_speed = 1.0f;         // Speed multiplier
    double _gravitational_constant = 1.0; // G constant
};

} // namespace gui
