// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include <gui/plot.hpp>
#include "implot.h"

void plot_2d_path(const std::string& label, const std::vector<Eigen::Vector2d>& positions) {
    if (positions.empty())
        return;
    ImPlot::PlotLine(label.c_str(), &positions[0].x(), &positions[0].y(), positions.size(), ImPlotLineFlags_None, 0, sizeof(Eigen::Vector2d));
}
