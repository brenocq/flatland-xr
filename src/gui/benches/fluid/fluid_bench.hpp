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

    bool _first_render = true;

    // Simulation state
    std::vector<FluidParticle> _particles;

    // Container bounds
    double _container_min_x = 1.0;
    double _container_max_x = 9.0;
    double _container_min_y = 1.0;
    double _container_max_y = 9.0;

    // Simulation parameters
    int _particle_count = 1000;
    double _particle_spacing = 0.1;
};

} // namespace gui
