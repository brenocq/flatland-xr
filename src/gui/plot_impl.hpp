// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#pragma once

#include "implot.h"
#include <gui/color.hpp>
#include <string>
#include <vector>

namespace gui {

template <typename Func> void plot_2d_func(const std::string& label, Func func, const Color& color, float weight) {
    ImPlotRect limits = ImPlot::GetPlotLimits();
    double min_x = limits.X.Min;
    double max_x = limits.X.Max;
    double min_y = limits.Y.Min;
    double max_y = limits.Y.Max;
    ImVec2 size = ImPlot::GetPlotSize();
    int num_pixels = static_cast<int>(size.x);

    if (num_pixels <= 1)
        return;

    // Limit resolution to avoid performance issues
    if (num_pixels > 4000)
        num_pixels = 4000;

    std::vector<double> xs;
    std::vector<double> ys;
    xs.reserve(num_pixels);
    ys.reserve(num_pixels);

    auto flush_segment = [&]() {
        if (!xs.empty()) {
            ImPlot::SetNextLineStyle(color, weight);
            ImPlot::PlotLine(label.c_str(), xs.data(), ys.data(), static_cast<int>(xs.size()));
            xs.clear();
            ys.clear();
        }
    };

    for (int i = 0; i < num_pixels; ++i) {
        double t = static_cast<double>(i) / static_cast<double>(num_pixels - 1);
        double x = min_x + t * (max_x - min_x);
        double y = func(x);

        xs.push_back(x);
        ys.push_back(y);
        if (y < min_y || y > max_y)
            flush_segment();
    }
    flush_segment();
}

template <int Size> void plot_vector(const std::string& label, const std::vector<float>& time_axis,
                                     const std::vector<Eigen::Vector<float, Size>>& data, const std::vector<Color>& colors, float line_width) {
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

template <int Size> void plot_covariance(const std::string& label, const std::vector<float>& time_axis,
                                         const std::vector<Eigen::Vector<float, Size>>& mean, const std::vector<Eigen::Vector<float, Size>>& std_dev,
                                         const std::vector<Color>& colors) {
    if (mean.empty() || std_dev.empty() || time_axis.empty())
        return;

    const size_t n = std::min({mean.size(), std_dev.size(), time_axis.size()});

    // Component labels: X, Y, Z, W for size <= 4, otherwise 0, 1, 2, 3, ...
    const char* component_names[] = {"X", "Y", "Z", "W"};

    // Plot each dimension as a shaded region with sigma lines
    for (int dim = 0; dim < Size; ++dim) {
        // Extract component data
        std::vector<float> mean_comp(n), upper_3sigma(n), lower_3sigma(n);

        for (size_t i = 0; i < n; ++i) {
            mean_comp[i] = mean[i](dim);
            float sigma = std_dev[i](dim);
            upper_3sigma[i] = mean_comp[i] + 3.0f * sigma;
            lower_3sigma[i] = mean_comp[i] - 3.0f * sigma;
        }

        // Generate label
        std::string dim_label;
        if (Size <= 4) {
            dim_label = label + " " + component_names[dim] + " Cov";
        } else {
            dim_label = label + " " + std::to_string(dim) + " Cov";
        }

        // Get color for this dimension (cycle through colors if needed)
        Color color = colors[dim % colors.size()];
        Color shaded_color = color;
        shaded_color.a() = 0.1f; // Set alpha to 0.1 for transparency

        // Plot 3-sigma shaded region
        ImPlot::SetNextFillStyle(ImVec4(shaded_color));
        ImPlot::PlotShaded(dim_label.c_str(), time_axis.data(), lower_3sigma.data(), upper_3sigma.data(), static_cast<int>(n));
    }
}

} // namespace gui
