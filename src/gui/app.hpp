// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#pragma once

#include <gui/benches/fluid/fluid_bench.hpp>
#include <gui/benches/gravity/gravity_bench.hpp>
#include <gui/benches/xr/xr_bench.hpp>

namespace gui {

class App {
  public:
    App();

    /// Startup app internal data structures
    void startup();

    /// Shutdown the app and cleanup internal data structures
    void shutdown();

    /// Update app logic and render frame
    void update();

  private:
    /// Setup initial docking layout
    void setup_docking_layout(ImGuiID dockspace_id);

    /// Render the menu bar
    void render_menu_bar();

    /// Render the about window
    void render_about_window();

    bool _first_render = true;

    //----- Menu state -----//
    bool _show_imgui_demo = false;
    bool _show_implot_demo = false;
    bool _show_about = false;

    //----- Physics benches -----//
    XRBench _xr_bench;
    GravityBench _gravity_bench;
    FluidBench _fluid_bench;
};

} // namespace gui
