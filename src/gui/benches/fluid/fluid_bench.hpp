// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#pragma once

#include <gui/benches/bench.hpp>

namespace gui {

/// Fluid dynamics bench to simulate fluid flow
class FluidBench : public Bench {
  public:
    FluidBench();

    void render() override;

  private:
    /// Render the configuration panel
    void render_config_panel();

    bool _first_render = true;
};

} // namespace gui
