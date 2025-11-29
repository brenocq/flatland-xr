// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

// MSCKF Playground

#include "imgui.h"
#include "implot.h"
#include <gui/window.hpp>

int main() {
    Window window("MSCKF Playground", 1200, 800);

    if (!window.create())
        return -1;

    while (!window.should_close()) {
        window.begin_frame();

        ImGui::ShowDemoWindow();
        ImPlot::ShowDemoWindow();

        window.end_frame();
    }
    window.destroy();

    return 0;
}
