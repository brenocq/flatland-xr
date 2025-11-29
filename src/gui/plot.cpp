// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include "implot.h"
#include <gui/plot.hpp>

void plot_2d_path(const std::string& label, const std::vector<Eigen::Vector2f>& positions, const Color& color, float weight) {
    if (positions.empty())
        return;

    ImPlot::SetNextLineStyle(ImVec4(color), weight);
    ImPlot::PlotLine(label.c_str(), &positions[0].x(), &positions[0].y(), positions.size(), ImPlotLineFlags_None, 0, sizeof(Eigen::Vector2f));
}
