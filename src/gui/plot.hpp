// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#pragma once

#include <Eigen/Dense>
#include <gui/color.hpp>
#include <string>
#include <vector>

void plot_2d_path(const std::string& label, const std::vector<Eigen::Vector2f>& positions, const Color& color = Color::Auto(), float weight = -1.0f);
