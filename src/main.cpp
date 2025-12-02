// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include <gui/app.hpp>
#include <gui/window.hpp>

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

// Global state for Emscripten main loop
struct AppState {
    gui::Window* window;
    gui::App* app;
};

#ifdef __EMSCRIPTEN__
// Main loop iteration for Emscripten
void main_loop_iteration(void* arg) {
    AppState* state = static_cast<AppState*>(arg);

    if (state->window->should_close()) {
        // Cleanup and cancel main loop
        state->app->shutdown();
        state->window->destroy();
        emscripten_cancel_main_loop();
        delete state->app;
        delete state->window;
        delete state;
        return;
    }

    state->window->begin_frame();
    state->app->update();
    state->window->end_frame();
}
#endif

int main() {
    gui::Window* window = new gui::Window("Flatland XR", 1200, 800);
    gui::App* app = new gui::App();

    // Setup
    if (!window->create()) {
        delete app;
        delete window;
        return -1;
    }
    app->startup();

#ifdef __EMSCRIPTEN__
    // Emscripten: Use emscripten_set_main_loop
    AppState* state = new AppState{window, app};
    emscripten_set_main_loop_arg(main_loop_iteration, state, 0, 1);
#else
    // Native: Use traditional while loop
    while (!window->should_close()) {
        window->begin_frame();
        app->update();
        window->end_frame();
    }

    // Cleanup
    app->shutdown();
    window->destroy();
    delete app;
    delete window;
#endif

    return 0;
}
