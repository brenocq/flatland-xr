// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include "imgui.h"
#include "implot.h"
#include <cmath>
#include <gui/benches/fluid/fluid_bench.hpp>
#include <gui/color.hpp>
#include <gui/plot.hpp>

namespace gui {

FluidBench::FluidBench() : Bench("Fluid"), _spatial_grid(0.2) { initialize_particles(); }

void FluidBench::initialize_particles() {
    _particles.clear();

    // Calculate container center
    double center_x = (_container_min_x + _container_max_x) / 2.0;
    double center_y = (_container_min_y + _container_max_y) / 2.0;

    // Calculate radius for the circle based on particle count and spacing
    // Area of circle = π*r², particles needed = area / spacing²
    double particles_per_area = 1.0 / (_particle_spacing * _particle_spacing);
    double circle_area = _particle_count / particles_per_area;
    double circle_radius = std::sqrt(circle_area / M_PI);

    // Generate particles in a grid pattern within a circle
    int particles_per_row = static_cast<int>(2.0 * circle_radius / _particle_spacing);
    int num_rows = particles_per_row;

    for (int row = 0; row < num_rows; row++) {
        for (int col = 0; col < particles_per_row; col++) {
            double x = center_x - circle_radius + (col + 0.5) * _particle_spacing;
            double y = center_y - circle_radius + (row + 0.5) * _particle_spacing;

            // Check if point is inside the circle
            double dx = x - center_x;
            double dy = y - center_y;
            double dist_from_center = std::sqrt(dx * dx + dy * dy);

            if (dist_from_center <= circle_radius) {
                _particles.emplace_back(x, y);

                // Stop if we've reached the desired count
                if (_particles.size() >= static_cast<size_t>(_particle_count)) {
                    _initial_particles = _particles;
                    return;
                }
            }
        }
    }

    // Store initial state
    _initial_particles = _particles;
}

void FluidBench::reset_simulation() {
    _particles = _initial_particles;
    _simulation_time = 0.0;
    _time_accumulator = 0.0;
    _is_playing = false;
}

void FluidBench::build_spatial_grid() {
    // Update grid cell size to match current smoothing radius
    _spatial_grid.set_cell_size(_smoothing_radius);

    _spatial_grid.clear();
    for (size_t i = 0; i < _particles.size(); i++) {
        _spatial_grid.insert(i, _particles[i].position);
    }
}

// SPH Kernel Functions
double FluidBench::kernel_poly6(double r, double h) const {
    if (r < 0.0 || r >= h) {
        return 0.0;
    }

    // Poly6 kernel: W(r,h) = (315/(64πh^9)) * (h² - r²)³
    const double h2 = h * h;
    const double h9 = h2 * h2 * h2 * h2 * h;
    const double coefficient = 315.0 / (64.0 * M_PI * h9);
    const double diff = h2 - r * r;

    return coefficient * diff * diff * diff;
}

Eigen::Vector2d FluidBench::kernel_spiky_gradient(const Eigen::Vector2d& r_vec, double r, double h) const {
    if (r <= 0.0 || r >= h) {
        return Eigen::Vector2d(0.0, 0.0);
    }

    // Spiky gradient: ∇W(r,h) = -(45/(πh^6)) * (h-r)² * (r_vec/r)
    const double h6 = h * h * h * h * h * h;
    const double coefficient = -45.0 / (M_PI * h6);
    const double h_minus_r = h - r;
    const double scale = coefficient * h_minus_r * h_minus_r / r;

    return scale * r_vec;
}

double FluidBench::kernel_viscosity_laplacian(double r, double h) const {
    if (r < 0.0 || r >= h) {
        return 0.0;
    }

    // Viscosity Laplacian: ∇²W(r,h) = (45/(πh^6)) * (h-r)
    const double h6 = h * h * h * h * h * h;
    const double coefficient = 45.0 / (M_PI * h6);

    return coefficient * (h - r);
}

// SPH Force Computation
void FluidBench::compute_density() {
    // Compute density for each particle using SPH interpolation
    for (size_t i = 0; i < _particles.size(); i++) {
        auto& pi = _particles[i];
        double density = 0.0;

        // Find neighbors within smoothing radius
        auto neighbors = _spatial_grid.query_neighbors(pi.position, _smoothing_radius);

        for (size_t j : neighbors) {
            const auto& pj = _particles[j];
            Eigen::Vector2d r_vec = pj.position - pi.position;
            double r = r_vec.norm();

            // ρᵢ = Σⱼ mⱼ W(rᵢⱼ, h)
            density += _particle_mass * kernel_poly6(r, _smoothing_radius);
        }

        // Clamp density to avoid instabilities
        pi.density = std::max(density, _rest_density * 0.5);
    }
}

void FluidBench::compute_pressure() {
    // Compute pressure using equation of state: p = k(ρ - ρ₀)
    for (auto& p : _particles) {
        // Only apply pressure when density exceeds rest density (no negative pressure)
        p.pressure = std::max(0.0, _gas_constant * (p.density - _rest_density));
    }
}

void FluidBench::compute_pressure_forces() {
    // Compute pressure forces for each particle
    for (size_t i = 0; i < _particles.size(); i++) {
        auto& pi = _particles[i];
        Eigen::Vector2d pressure_force(0.0, 0.0);

        // Find neighbors within smoothing radius
        auto neighbors = _spatial_grid.query_neighbors(pi.position, _smoothing_radius);

        for (size_t j : neighbors) {
            if (i == j)
                continue; // Skip self

            const auto& pj = _particles[j];
            Eigen::Vector2d r_vec = pj.position - pi.position;
            double r = r_vec.norm();

            if (r > 0.0 && r < _smoothing_radius) {
                // Symmetric pressure force: fᵢ = -Σⱼ mⱼ (pᵢ + pⱼ)/(2ρⱼ) ∇W(rᵢⱼ, h)
                double density_j = std::max(pj.density, _rest_density * 0.5); // Prevent division by very small density
                double pressure_term = (pi.pressure + pj.pressure) / (2.0 * density_j);
                Eigen::Vector2d gradient = kernel_spiky_gradient(r_vec, r, _smoothing_radius);
                pressure_force -= _particle_mass * pressure_term * gradient;
            }
        }

        pi.force += pressure_force;
    }
}

void FluidBench::compute_viscosity_forces() {
    // Compute viscosity forces for each particle
    for (size_t i = 0; i < _particles.size(); i++) {
        auto& pi = _particles[i];
        Eigen::Vector2d viscosity_force(0.0, 0.0);

        // Find neighbors within smoothing radius
        auto neighbors = _spatial_grid.query_neighbors(pi.position, _smoothing_radius);

        for (size_t j : neighbors) {
            if (i == j)
                continue; // Skip self

            const auto& pj = _particles[j];
            Eigen::Vector2d r_vec = pj.position - pi.position;
            double r = r_vec.norm();

            if (r > 0.0 && r < _smoothing_radius) {
                // Viscosity force: fᵢ = μ Σⱼ mⱼ (vⱼ - vᵢ)/ρⱼ ∇²W(rᵢⱼ, h)
                Eigen::Vector2d velocity_diff = pj.velocity - pi.velocity;
                double laplacian = kernel_viscosity_laplacian(r, _smoothing_radius);
                double density_j = std::max(pj.density, _rest_density * 0.5); // Prevent division by very small density
                viscosity_force += _viscosity * _particle_mass * (velocity_diff / density_j) * laplacian;
            }
        }

        pi.force += viscosity_force;
    }
}

void FluidBench::simulate_step(double dt) {
    // Build spatial grid for neighbor queries
    build_spatial_grid();

    // Compute SPH quantities
    compute_density();
    compute_pressure();

    // Clear forces
    for (auto& p : _particles) {
        p.force = Eigen::Vector2d(0.0, 0.0);
    }

    // Apply gravity
    for (auto& p : _particles) {
        p.force.y() += _gravity * _particle_mass;
    }

    // Compute SPH forces
    compute_pressure_forces();
    compute_viscosity_forces();

    // Integrate forces to update velocities and positions
    for (auto& p : _particles) {
        // Semi-implicit Euler integration
        p.velocity += (p.force / _particle_mass) * dt;
        p.position += p.velocity * dt;

        // Boundary collision with damping
        if (p.position.x() < _container_min_x) {
            p.position.x() = _container_min_x;
            p.velocity.x() *= -_damping;
        }
        if (p.position.x() > _container_max_x) {
            p.position.x() = _container_max_x;
            p.velocity.x() *= -_damping;
        }
        if (p.position.y() < _container_min_y) {
            p.position.y() = _container_min_y;
            p.velocity.y() *= -_damping;
        }
        if (p.position.y() > _container_max_y) {
            p.position.y() = _container_max_y;
            p.velocity.y() *= -_damping;
        }
    }

    _simulation_time += dt;
}

void FluidBench::render_config_panel() {
    ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Simulation");
    ImGui::Separator();

    // Play/Pause button
    if (_is_playing) {
        if (ImGui::Button("⏸ Pause", ImVec2(120, 0))) {
            _is_playing = false;
        }
    } else {
        if (ImGui::Button("▶ Play", ImVec2(120, 0))) {
            _is_playing = true;
        }
    }

    ImGui::SameLine();
    if (ImGui::Button("⏭ Step", ImVec2(120, 0))) {
        simulate_step(_time_step);
    }

    ImGui::SameLine();
    if (ImGui::Button("⟲ Reset", ImVec2(120, 0))) {
        reset_simulation();
    }

    ImGui::Text("Time: %.3f s", _simulation_time);
    ImGui::Text("Particles: %zu", _particles.size());

    ImGui::SliderFloat("Time Step (dt)", &_time_step, 0.0001f, 0.02f, "%.4f");
    ImGui::SliderFloat("Playback Speed", &_playback_speed, 0.001f, 5.0f, "%.3fx");

    if (ImGui::SliderInt("Particle Count", &_particle_count, 100, 5000)) {
        initialize_particles();
    }

    ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Physics");
    ImGui::Separator();

    ImGui::DragScalar("Gravity", ImGuiDataType_Double, &_gravity, 0.1f, nullptr, nullptr, "%.2f");
    ImGui::DragScalar("Damping", ImGuiDataType_Double, &_damping, 0.01f, nullptr, nullptr, "%.2f");

    ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "SPH Parameters");
    ImGui::Separator();

    ImGui::DragScalar("Smoothing Radius", ImGuiDataType_Double, &_smoothing_radius, 0.01f, nullptr, nullptr, "%.3f");
    ImGui::DragScalar("Particle Mass", ImGuiDataType_Double, &_particle_mass, 0.1f, nullptr, nullptr, "%.2f");
    ImGui::DragScalar("Rest Density", ImGuiDataType_Double, &_rest_density, 10.0f, nullptr, nullptr, "%.1f");
    ImGui::DragScalar("Gas Constant", ImGuiDataType_Double, &_gas_constant, 10.0f, nullptr, nullptr, "%.1f");
    ImGui::DragScalar("Viscosity", ImGuiDataType_Double, &_viscosity, 0.01f, nullptr, nullptr, "%.3f");

    ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Container");
    ImGui::Separator();

    bool container_changed = false;
    container_changed |= ImGui::DragScalar("Min X", ImGuiDataType_Double, &_container_min_x, 0.1f);
    container_changed |= ImGui::DragScalar("Max X", ImGuiDataType_Double, &_container_max_x, 0.1f);
    container_changed |= ImGui::DragScalar("Min Y", ImGuiDataType_Double, &_container_min_y, 0.1f);
    container_changed |= ImGui::DragScalar("Max Y", ImGuiDataType_Double, &_container_max_y, 0.1f);

    if (container_changed) {
        initialize_particles();
    }
}

void FluidBench::render() {
    if (!ImGui::Begin("Fluid Bench")) {
        ImGui::End();
        return;
    }

    // Update simulation if playing (fixed timestep with accumulator)
    if (_is_playing) {
        // Accumulate frame time scaled by playback speed
        _time_accumulator += ImGui::GetIO().DeltaTime * _playback_speed;

        // Take as many steps as needed to consume accumulated time
        while (_time_accumulator >= _time_step) {
            simulate_step(_time_step);
            _time_accumulator -= _time_step;
        }
    }

    // Left side: Main visualization
    {
        ImGui::BeginChild("left pane", ImVec2(ImGui::GetContentRegionAvail().x * 0.6f, 0), ImGuiChildFlags_None);

        if (ImPlot::BeginPlot("Fluid Simulation", ImVec2(-1, -1), ImPlotFlags_Equal)) {
            ImPlot::SetupAxes("X", "Y");
            ImPlot::SetupAxisLimits(ImAxis_X1, 0.0, 10.0);
            ImPlot::SetupAxisLimits(ImAxis_Y1, 0.0, 10.0);

            // Draw container boundary
            std::vector<Eigen::Vector2f> container_boundary = {
                Eigen::Vector2f(_container_min_x, _container_min_y), Eigen::Vector2f(_container_max_x, _container_min_y),
                Eigen::Vector2f(_container_max_x, _container_max_y), Eigen::Vector2f(_container_min_x, _container_max_y),
                Eigen::Vector2f(_container_min_x, _container_min_y)};

            plot_2d_line("Container", container_boundary, Color::White(), 2.0f);

            // Draw particles
            if (!_particles.empty()) {
                std::vector<Eigen::Vector2f> particle_positions;
                particle_positions.reserve(_particles.size());

                for (const auto& p : _particles) {
                    particle_positions.emplace_back(p.position.cast<float>());
                }

                plot_2d_scatter("Particles", particle_positions, Color::CatBlue(), 3.0f);
            }

            ImPlot::EndPlot();
        }

        ImGui::EndChild();
    }

    ImGui::SameLine();

    // Right side: Config panel
    {
        ImGui::BeginChild("config pane", ImVec2(0, 0), ImGuiChildFlags_Borders);
        render_config_panel();
        ImGui::EndChild();
    }

    ImGui::End();
}

} // namespace gui
