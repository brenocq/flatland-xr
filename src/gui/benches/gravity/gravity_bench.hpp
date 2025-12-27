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

    GravityPreset _current_preset = GravityPreset::TwinStars;
    std::vector<GravityBody> _bodies;
};

} // namespace gui
