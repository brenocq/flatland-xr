// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include "imgui.h"
#include "implot.h"
#include <cmath>
#include <gui/benches/gravity/gravity_bench.hpp>
#include <gui/color.hpp>
#include <gui/plot.hpp>

namespace gui {

GravityBench::GravityBench() : Bench("Gravity") {
    _x.resize(2);
    _I.resize(2);

    _x[0] = -1.0;
    _I[0] = 2.0;

    _x[1] = 1.0;
    _I[1] = 1.0;
}

double GravityBench::surface_func(double x) {
    size_t n = _x.size();
    double y = 0;
    for (size_t i = 0; i < n; ++i) {
        double Ii = _I[i];
        double xi = _x[i];
        double r = std::min((x - xi), 0.001); // Softening factor
        y -= Ii / (r * r);
    }
    return y;
}

void GravityBench::render() {
    if (!ImGui::Begin("Gravity Bench")) {
        ImGui::End();
        return;
    }

    if (ImPlot::BeginPlot("Orbital Mechanics", ImVec2(-1, -1), ImPlotFlags_Equal)) {
        ImPlot::SetupAxes("X", "Y");
        ImPlot::SetupAxisLimits(ImAxis_X1, -3.0, 3.0);

        // Plot the surface function
        plot_2d_func("Surface", [this](double x) { return surface_func(x); }, Color::CatLavender(), 2.0f);

        ImPlot::EndPlot();
    }

    ImGui::End();
}

} // namespace gui
