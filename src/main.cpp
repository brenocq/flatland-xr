// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

// MSCKF Playground

#include <gui/app.hpp>
#include <gui/window.hpp>

int main() {
    Window window("MSCKF Playground", 1200, 800);
    App app;

    // Setup
    if (!window.create())
        return -1;
    app.startup();

    // Main loop
    while (!window.should_close()) {
        window.begin_frame();
        app.update();
        window.end_frame();
    }

    // Cleanup
    app.shutdown();
    window.destroy();

    return 0;
}
