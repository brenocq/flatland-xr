// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include "imgui.h"
#include "implot.h"
#include <cmath>
#include <gui/benches/gravity/gravity_bench.hpp>
#include <gui/color.hpp>
#include <gui/plot.hpp>
#include <gui/widgets/text.hpp>

namespace gui {

GravityBench::GravityBench() : Bench("Gravity") { load_preset(GravityPreset::TwinStars); }

void GravityBench::load_preset(GravityPreset preset) {
    _current_preset = preset;
    _bodies.clear();

    switch (preset) {
        case GravityPreset::TwinStars:
            _bodies.emplace_back(1.0, Eigen::Vector2d(-1.0, 0.0), Eigen::Vector2d(0.0, 0.3));
            _bodies.emplace_back(1.0, Eigen::Vector2d(1.0, 0.0), Eigen::Vector2d(0.0, -0.3));
            break;

        case GravityPreset::SolarSystem:
            // Sun
            _bodies.emplace_back(10.0, Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0.0, 0.0));
            // Inner planets
            _bodies.emplace_back(0.1, Eigen::Vector2d(1.0, 0.0), Eigen::Vector2d(0.0, 3.0));
            _bodies.emplace_back(0.15, Eigen::Vector2d(1.5, 0.0), Eigen::Vector2d(0.0, 2.4));
            // Outer planets
            _bodies.emplace_back(0.3, Eigen::Vector2d(2.5, 0.0), Eigen::Vector2d(0.0, 1.8));
            _bodies.emplace_back(0.4, Eigen::Vector2d(3.5, 0.0), Eigen::Vector2d(0.0, 1.5));
            break;

        case GravityPreset::ThreeBody:
            _bodies.emplace_back(1.0, Eigen::Vector2d(-1.0, 0.0), Eigen::Vector2d(0.0, 0.5));
            _bodies.emplace_back(1.0, Eigen::Vector2d(1.0, 0.0), Eigen::Vector2d(0.0, -0.5));
            _bodies.emplace_back(0.5, Eigen::Vector2d(0.0, 1.5), Eigen::Vector2d(-0.3, 0.0));
            break;

        case GravityPreset::OrbitingPlanets:
            // Central star
            _bodies.emplace_back(5.0, Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0.0, 0.0));
            // Two planets in different orbits
            _bodies.emplace_back(0.2, Eigen::Vector2d(2.0, 0.0), Eigen::Vector2d(0.0, 1.5));
            _bodies.emplace_back(0.2, Eigen::Vector2d(0.0, 2.5), Eigen::Vector2d(-1.3, 0.0));
            break;

        case GravityPreset::Custom:
            // Start with one body
            _bodies.emplace_back(1.0, Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0.0, 0.0));
            break;
    }

    // Store initial conditions
    _initial_bodies = _bodies;
    _simulation_time = 0.0;
}

void GravityBench::reset_simulation() {
    _bodies = _initial_bodies;
    _simulation_time = 0.0;
    _is_playing = false;
}

void GravityBench::simulate_step(double dt) {
    const double softening = 0.01; // Softening factor to avoid singularities

    // Compute accelerations for all bodies
    std::vector<Eigen::Vector2d> accelerations(_bodies.size(), Eigen::Vector2d(0.0, 0.0));

    for (size_t i = 0; i < _bodies.size(); i++) {
        for (size_t j = 0; j < _bodies.size(); j++) {
            if (i == j)
                continue;

            Eigen::Vector2d r = _bodies[j].position - _bodies[i].position;
            double dist_sq = r.squaredNorm() + softening * softening;
            double dist = std::sqrt(dist_sq);
            double force_magnitude = _gravitational_constant * _bodies[j].mass / dist_sq;

            accelerations[i] += force_magnitude * (r / dist);
        }
    }

    // Update velocities and positions (simple Euler integration)
    for (size_t i = 0; i < _bodies.size(); i++) {
        _bodies[i].velocity += accelerations[i] * dt;
        _bodies[i].position += _bodies[i].velocity * dt;
    }

    _simulation_time += dt;
}

double GravityBench::potential_func(double x, double y) {
    double potential = 0.0;
    const double softening = 0.1; // Softening factor to avoid singularities

    for (const auto& body : _bodies) {
        double dx = x - body.position.x();
        double dy = y - body.position.y();
        double r = std::sqrt(dx * dx + dy * dy + softening * softening);
        potential -= body.mass / r;
    }

    return potential;
}

void GravityBench::render_config_panel() {
    // Preset selection
    ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Setup");
    ImGui::Separator();

    const char* preset_names[] = {"Custom", "Twin Stars", "Solar System", "Three Body", "Orbiting Planets"};
    int current_preset_idx = static_cast<int>(_current_preset);

    if (ImGui::Combo("Preset", &current_preset_idx, preset_names, IM_ARRAYSIZE(preset_names))) {
        load_preset(static_cast<GravityPreset>(current_preset_idx));
    }

    // Simulation controls
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
    if (ImGui::Button("⟲ Reset", ImVec2(120, 0))) {
        reset_simulation();
    }

    ImGui::Text("Time: %.2f", _simulation_time);
    ImGui::SliderFloat("Speed", &_playback_speed, 0.1f, 5.0f, "%.1fx");
    ImGui::SliderFloat("Time Step", &_time_step, 0.0001f, 0.1f, "%.4f");
    ImGui::DragScalar("G Constant", ImGuiDataType_Double, &_gravitational_constant, 0.1f, nullptr, nullptr, "%.2f");

    ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Bodies: %zu", _bodies.size());
    ImGui::Separator();

    if (ImGui::Button("Add Body")) {
        _bodies.emplace_back();
        _initial_bodies = _bodies;
        _current_preset = GravityPreset::Custom;
    }

    ImGui::SameLine();
    if (ImGui::Button("Remove Body") && !_bodies.empty()) {
        _bodies.pop_back();
        _initial_bodies = _bodies;
        _current_preset = GravityPreset::Custom;
    }

    ImGui::Separator();

    // Body properties
    for (size_t i = 0; i < _bodies.size(); i++) {
        ImGui::PushID(static_cast<int>(i));

        // Get color for this body
        Color body_color = Color::GetPaletteColor(i);

        // Color the tree node text
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(body_color));
        bool node_open = ImGui::TreeNode("Body", "Body %zu", i);
        ImGui::PopStyleColor();

        if (node_open) {
            if (ImGui::DragScalar("Mass", ImGuiDataType_Double, &_bodies[i].mass, 0.1f, nullptr, nullptr, "%.2f")) {
                _current_preset = GravityPreset::Custom;
            }

            float pos[2] = {static_cast<float>(_bodies[i].position.x()), static_cast<float>(_bodies[i].position.y())};
            if (ImGui::DragFloat2("Position", pos, 0.1f)) {
                _bodies[i].position = Eigen::Vector2d(pos[0], pos[1]);
                _current_preset = GravityPreset::Custom;
            }

            float vel[2] = {static_cast<float>(_bodies[i].velocity.x()), static_cast<float>(_bodies[i].velocity.y())};
            if (ImGui::DragFloat2("Velocity", vel, 0.1f)) {
                _bodies[i].velocity = Eigen::Vector2d(vel[0], vel[1]);
                _current_preset = GravityPreset::Custom;
            }

            ImGui::TreePop();
        }
        ImGui::PopID();
    }
}

void GravityBench::render() {
    if (!ImGui::Begin("Gravity Bench")) {
        ImGui::End();
        return;
    }

    // Update simulation if playing
    if (_is_playing) {
        // Use ImGui delta time and apply playback speed
        float frame_dt = ImGui::GetIO().DeltaTime * _playback_speed;
        // Multiple sub-steps for stability
        int num_substeps = std::max(1, static_cast<int>(frame_dt / _time_step));
        for (int i = 0; i < num_substeps; i++) {
            simulate_step(_time_step);
        }
    }

    // Left side: Main visualization
    {
        ImGui::BeginChild("left pane", ImVec2(ImGui::GetContentRegionAvail().x * 0.75f, 0), ImGuiChildFlags_None);

        if (ImPlot::BeginPlot("Gravitational Potential Field", ImVec2(-1, -1), ImPlotFlags_Equal)) {
            ImPlot::SetupAxes("X", "Y");
            ImPlot::SetupAxisLimits(ImAxis_X1, -5.0, 5.0);
            ImPlot::SetupAxisLimits(ImAxis_Y1, -5.0, 5.0);

            // Plot bodies with colors from the palette
            for (size_t i = 0; i < _bodies.size(); i++) {
                double x = _bodies[i].position.x();
                double y = _bodies[i].position.y();
                double radius = std::pow(_bodies[i].mass, 0.33) * 0.2; // Size based on mass

                // Get color from palette
                Color body_color = Color::GetPaletteColor(i);
                ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, radius * 10, ImVec4(body_color), IMPLOT_AUTO, ImVec4(body_color));

                char label[32];
                snprintf(label, sizeof(label), "Body %zu", i);
                ImPlot::PlotScatter(label, &x, &y, 1);

                // Plot velocity arrow
                Eigen::Vector2f pos = _bodies[i].position.cast<float>();
                Eigen::Vector2f vel = _bodies[i].velocity.cast<float>();
                if (vel.norm() > 1e-6f) {
                    char arrow_label[64];
                    snprintf(arrow_label, sizeof(arrow_label), "Velocity %zu", i);
                    plot_2d_arrow(arrow_label, pos, vel, body_color, 1.0f, std::min(vel.norm() * 0.2f, 0.2f));
                }
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
