// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#pragma once

#include <Eigen/Dense>
#include <gui/benches/bench.hpp>
#include <vector>

namespace gui {

/// A fluid particle with SPH properties
struct FluidParticle {
    Eigen::Vector2d position;
    Eigen::Vector2d velocity;
    double density;
    double pressure;

    FluidParticle() : position(0.0, 0.0), velocity(0.0, 0.0), density(0.0), pressure(0.0) {}
    FluidParticle(double x, double y) : position(x, y), velocity(0.0, 0.0), density(0.0), pressure(0.0) {}
};

/// Fluid dynamics bench to simulate fluid flow
class FluidBench : public Bench {
  public:
    FluidBench();

    void render() override;

  private:
    /// Render the configuration panel
    void render_config_panel();

    /// Initialize particles in a grid
    void initialize_particles();

    /// Reset simulation to initial state
    void reset_simulation();

    /// Simulate one time step
    void simulate_step(double dt);

    bool _first_render = true;

    // Simulation state
    std::vector<FluidParticle> _particles;
    std::vector<FluidParticle> _initial_particles; // For reset
    bool _is_playing = false;
    double _simulation_time = 0.0;
    double _time_accumulator = 0.0; // Accumulate frame time for fixed timestep

    // Container bounds
    double _container_min_x = 1.0;
    double _container_max_x = 9.0;
    double _container_min_y = 1.0;
    double _container_max_y = 9.0;

    // Simulation parameters
    int _particle_count = 1000;
    double _particle_spacing = 0.1;
    float _time_step = 0.005f;    // dt for integration
    float _playback_speed = 1.0f; // Speed multiplier
    double _gravity = -9.81;      // Gravity acceleration (downward)
    double _damping = 0.95;       // Velocity damping on collision
};

} // namespace gui
