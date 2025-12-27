// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#pragma once

#include <gui/benches/bench.hpp>
#include <vector>

namespace gui {

/// Gravity bench to simulate gravity effects
class GravityBench : public Bench {
  public:
    GravityBench();

    void render() override;

  private:
    double surface_func(double x);

    std::vector<double> _x; // State of each particle
    std::vector<double> _I; // Inertia of each particle
};

} // namespace gui
