// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#pragma once

#include <string>

struct GLFWwindow;

class Window {
  public:
    Window(const std::string& name, size_t width, size_t height);

    /// Create the OS window
    bool create();

    /// Destroy the OS window
    void destroy();

    /// Return true if the OS window should be destroyed
    bool should_close();

    /// Prepare frame for rendering and get mouse/keyboard inputs
    void begin_frame();

    // Render the current frame and display it on the OS window
    void end_frame();

  private:
    std::string _name;
    size_t _width;
    size_t _height;

    GLFWwindow* _window;
};
