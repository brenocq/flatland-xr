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

    // Calculate grid dimensions based on particle count and spacing
    double container_width = _container_max_x - _container_min_x;
    double container_height = _container_max_y - _container_min_y;

    // Fill bottom half of container with particles in a grid
    double spawn_height = container_height * 0.5;
    int particles_per_row = static_cast<int>(container_width / _particle_spacing);
    int num_rows = static_cast<int>(spawn_height / _particle_spacing);

    // Adjust to match desired particle count
    int total_grid_particles = particles_per_row * num_rows;
    if (total_grid_particles == 0)
        total_grid_particles = 1;

    double scale = std::sqrt(static_cast<double>(_particle_count) / total_grid_particles);
    particles_per_row = std::max(1, static_cast<int>(particles_per_row * scale));
    num_rows = std::max(1, static_cast<int>(num_rows * scale));

    // Recalculate spacing to fit particles nicely
    double actual_spacing_x = container_width / (particles_per_row + 1);
    double actual_spacing_y = spawn_height / (num_rows + 1);

    // Spawn particles in grid
    for (int row = 0; row < num_rows; row++) {
        for (int col = 0; col < particles_per_row; col++) {
            double x = _container_min_x + (col + 1) * actual_spacing_x;
            double y = _container_min_y + (row + 1) * actual_spacing_y;
            _particles.emplace_back(x, y);

            // Stop if we've reached the desired count
            if (_particles.size() >= static_cast<size_t>(_particle_count)) {
                _initial_particles = _particles;
                return;
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

void FluidBench::simulate_step(double dt) {
    // Build spatial grid for neighbor queries
    build_spatial_grid();

    // TODO: Compute SPH forces (density, pressure, viscosity)
    // For now, just apply gravity

    // Clear forces
    for (auto& p : _particles) {
        p.force = Eigen::Vector2d(0.0, 0.0);
    }

    // Apply gravity
    for (auto& p : _particles) {
        p.force.y() += _gravity * _particle_mass;
    }

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
