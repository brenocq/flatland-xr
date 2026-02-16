// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#pragma once

#include <Eigen/Dense>
#include <gui/benches/bench.hpp>
#include <unordered_map>
#include <vector>

namespace gui {

/// A fluid particle with SPH properties
struct FluidParticle {
    Eigen::Vector2d position;
    Eigen::Vector2d velocity;
    Eigen::Vector2d force; // Accumulated forces
    double density;
    double pressure;

    FluidParticle() : position(0.0, 0.0), velocity(0.0, 0.0), force(0.0, 0.0), density(0.0), pressure(0.0) {}
    FluidParticle(double x, double y) : position(x, y), velocity(0.0, 0.0), force(0.0, 0.0), density(0.0), pressure(0.0) {}
};

/// Spatial hash grid for efficient neighbor search in 2D
class SpatialHashGrid {
  public:
    SpatialHashGrid(double cell_size) : _cell_size(cell_size) {}

    /// Update the cell size (should match smoothing radius)
    void set_cell_size(double cell_size) { _cell_size = cell_size; }

    /// Clear all particles from the grid
    void clear() { _grid.clear(); }

    /// Insert a particle at the given index
    void insert(size_t particle_idx, const Eigen::Vector2d& position) {
        int64_t key = hash(position);
        _grid[key].push_back(particle_idx);
    }

    /// Find all particles within the smoothing radius of the query position
    /// Returns indices of neighboring particles
    std::vector<size_t> query_neighbors(const Eigen::Vector2d& position, double /* radius */) const {
        std::vector<size_t> neighbors;

        // Check all neighboring cells (3x3 grid around the query position)
        int cx = static_cast<int>(std::floor(position.x() / _cell_size));
        int cy = static_cast<int>(std::floor(position.y() / _cell_size));

        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                int64_t key = hash_coords(cx + dx, cy + dy);
                auto it = _grid.find(key);
                if (it != _grid.end()) {
                    neighbors.insert(neighbors.end(), it->second.begin(), it->second.end());
                }
            }
        }

        return neighbors;
    }

  private:
    /// Hash a 2D position to a grid cell key
    int64_t hash(const Eigen::Vector2d& pos) const {
        int cx = static_cast<int>(std::floor(pos.x() / _cell_size));
        int cy = static_cast<int>(std::floor(pos.y() / _cell_size));
        return hash_coords(cx, cy);
    }

    /// Hash grid coordinates to a unique key
    int64_t hash_coords(int cx, int cy) const {
        // Simple spatial hash: combine x and y coordinates
        return (static_cast<int64_t>(cx) * 73856093) ^ (static_cast<int64_t>(cy) * 19349663);
    }

    double _cell_size;
    std::unordered_map<int64_t, std::vector<size_t>> _grid;
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

    /// Build spatial hash grid for current particle positions
    void build_spatial_grid();

    /// SPH Kernel Functions
    /// Poly6 kernel for density computation: W(r, h)
    double kernel_poly6(double r, double h) const;

    /// Spiky kernel gradient for pressure forces: ∇W(r, h)
    Eigen::Vector2d kernel_spiky_gradient(const Eigen::Vector2d& r_vec, double r, double h) const;

    /// Viscosity kernel Laplacian for viscosity forces: ∇²W(r, h)
    double kernel_viscosity_laplacian(double r, double h) const;

    /// Compute density for all particles
    void compute_density();

    /// Compute pressure for all particles
    void compute_pressure();

    /// Compute pressure forces for all particles
    void compute_pressure_forces();

    /// Compute viscosity forces for all particles
    void compute_viscosity_forces();

    bool _first_render = true;

    // Simulation state
    std::vector<FluidParticle> _particles;
    std::vector<FluidParticle> _initial_particles; // For reset
    SpatialHashGrid _spatial_grid;
    bool _is_playing = false;
    double _simulation_time = 0.0;
    double _time_accumulator = 0.0; // Accumulate frame time for fixed timestep

    // Container bounds
    double _container_min_x = 1.0;
    double _container_max_x = 9.0;
    double _container_min_y = 1.0;
    double _container_max_y = 9.0;

    // SPH parameters
    double _smoothing_radius = 0.2; // h - kernel support radius
    double _particle_mass = 0.02;   // m - mass per particle (reduced for stability)
    double _rest_density = 1000.0;  // ρ₀ - rest density
    double _gas_constant = 200.0;   // k - equation of state stiffness (reduced)
    double _viscosity = 0.5;        // μ - viscosity coefficient (increased)

    // Simulation parameters
    int _particle_count = 1000;
    double _particle_spacing = 0.1;
    float _time_step = 0.001f;    // dt for integration (reduced for stability)
    float _playback_speed = 1.0f; // Speed multiplier
    double _gravity = -9.81;      // Gravity acceleration (downward)
    double _damping = 0.95;       // Velocity damping on collision
};

} // namespace gui
