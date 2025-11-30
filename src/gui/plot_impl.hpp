// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#pragma once

#include "implot.h"
#include <gui/color.hpp>
#include <string>
#include <vector>

namespace gui {

template <int Size>
void plot_vector(const std::string& label, const std::vector<float>& time_axis, const std::vector<Eigen::Vector<float, Size>>& data,
                 const std::vector<Color>& colors, float line_width) {
    if (data.empty() || time_axis.empty())
        return;

    const size_t n = std::min(data.size(), time_axis.size());

    // Component labels: X, Y, Z, W for size <= 4, otherwise 0, 1, 2, 3, ...
    const char* component_names[] = {"X", "Y", "Z", "W"};

    // Plot each dimension as a separate line
    for (int dim = 0; dim < Size; ++dim) {
        // Extract component data
        std::vector<float> component(n);
        for (size_t i = 0; i < n; ++i) {
            component[i] = data[i](dim);
        }

        // Generate label
        std::string dim_label;
        if (Size <= 4) {
            dim_label = label + " " + component_names[dim];
        } else {
            dim_label = label + " " + std::to_string(dim);
        }

        // Get color for this dimension (cycle through colors if needed)
        Color color = colors[dim % colors.size()];

        // Set line style and plot
        if (line_width > 0.0f) {
            ImPlot::SetNextLineStyle(ImVec4(color), line_width);
        } else {
            ImPlot::SetNextLineStyle(ImVec4(color));
        }

        ImPlot::PlotLine(dim_label.c_str(), time_axis.data(), component.data(), static_cast<int>(n));
    }
}

} // namespace gui
