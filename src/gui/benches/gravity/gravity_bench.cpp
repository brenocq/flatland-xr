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
    ImGui::TextColored(Color::CatSapphire(), "Configuration");
    ImGui::Separator();

    // Preset selection
    const char* preset_names[] = {"Custom", "Twin Stars", "Solar System", "Three Body", "Orbiting Planets"};
    int current_preset_idx = static_cast<int>(_current_preset);

    if (ImGui::Combo("Preset", &current_preset_idx, preset_names, IM_ARRAYSIZE(preset_names))) {
        load_preset(static_cast<GravityPreset>(current_preset_idx));
    }

    ImGui::Separator();
    ImGui::Text("Bodies: %zu", _bodies.size());

    if (ImGui::Button("Add Body")) {
        _bodies.emplace_back();
        _current_preset = GravityPreset::Custom;
    }

    ImGui::SameLine();
    if (ImGui::Button("Remove Body") && !_bodies.empty()) {
        _bodies.pop_back();
        _current_preset = GravityPreset::Custom;
    }

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

            // Plot potential field lines (simplified)
            // const int num_lines = 10;
            // for (int i = 0; i < num_lines; i++) {
            //    double angle = 2.0 * M_PI * i / num_lines;
            //    std::vector<double> line_x, line_y;

            //    for (double r = 0.5; r < 5.0; r += 0.1) {
            //        double x = r * std::cos(angle);
            //        double y = r * std::sin(angle);
            //        line_x.push_back(x);
            //        line_y.push_back(y);
            //    }

            //    if (!line_x.empty()) {
            //        ImPlot::SetNextLineStyle(Color::CatSurface1(), 0.5f);
            //        ImPlot::PlotLine("##field", line_x.data(), line_y.data(), static_cast<int>(line_x.size()));
            //    }
            //}

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
