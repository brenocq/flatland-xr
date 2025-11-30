// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

// Silence OpenGL deprecation warnings on macOS
#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
#endif

#include <core/resources.hpp>
#include <gui/color.hpp>
#include <gui/window.hpp>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"
#include <GLFW/glfw3.h>
#include <iostream>

namespace gui {

Window::Window(const std::string& name, size_t width, size_t height) : _name(name), _width(width), _height(height) {}

// Callback to handle GLFW errors
void glfw_error_callback(int error, const char* description) { std::cerr << "GLFW Error " << error << ": " << description << '\n'; }

// Setup Catppuccin Mocha theme
void setup_catppuccin_style() {
    ImGuiStyle& style = ImGui::GetStyle();
    ImVec4* colors = style.Colors;

    // Catppuccin Mocha colors from Color class
    const ImVec4 base = ImVec4(Color::CatBase());
    const ImVec4 mantle = ImVec4(Color::CatMantle());
    const ImVec4 surface0 = ImVec4(Color::CatSurface0());
    const ImVec4 surface1 = ImVec4(Color::CatSurface1());
    const ImVec4 surface2 = ImVec4(Color::CatSurface2());
    const ImVec4 overlay0 = ImVec4(Color::CatOverlay0());
    const ImVec4 overlay2 = ImVec4(Color::CatOverlay2());
    const ImVec4 text = ImVec4(Color::CatText());
    const ImVec4 subtext0 = ImVec4(Color::CatSubtext0());
    const ImVec4 mauve = ImVec4(Color::CatMauve());
    const ImVec4 peach = ImVec4(Color::CatPeach());
    const ImVec4 yellow = ImVec4(Color::CatYellow());
    const ImVec4 green = ImVec4(Color::CatGreen());
    const ImVec4 teal = ImVec4(Color::CatTeal());
    const ImVec4 sapphire = ImVec4(Color::CatSapphire());
    const ImVec4 blue = ImVec4(Color::CatBlue());
    const ImVec4 lavender = ImVec4(Color::CatLavender());

    // Main window and backgrounds
    colors[ImGuiCol_WindowBg] = base;
    colors[ImGuiCol_ChildBg] = base;
    colors[ImGuiCol_PopupBg] = surface0;
    colors[ImGuiCol_Border] = surface1;
    colors[ImGuiCol_BorderShadow] = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);
    colors[ImGuiCol_FrameBg] = surface0;
    colors[ImGuiCol_FrameBgHovered] = surface1;
    colors[ImGuiCol_FrameBgActive] = surface2;
    colors[ImGuiCol_TitleBg] = mantle;
    colors[ImGuiCol_TitleBgActive] = surface0;
    colors[ImGuiCol_TitleBgCollapsed] = mantle;
    colors[ImGuiCol_MenuBarBg] = mantle;
    colors[ImGuiCol_ScrollbarBg] = surface0;
    colors[ImGuiCol_ScrollbarGrab] = surface2;
    colors[ImGuiCol_ScrollbarGrabHovered] = overlay0;
    colors[ImGuiCol_ScrollbarGrabActive] = overlay2;
    colors[ImGuiCol_CheckMark] = green;
    colors[ImGuiCol_SliderGrab] = sapphire;
    colors[ImGuiCol_SliderGrabActive] = blue;
    colors[ImGuiCol_Button] = surface0;
    colors[ImGuiCol_ButtonHovered] = surface1;
    colors[ImGuiCol_ButtonActive] = surface2;
    colors[ImGuiCol_Header] = surface0;
    colors[ImGuiCol_HeaderHovered] = surface1;
    colors[ImGuiCol_HeaderActive] = surface2;
    colors[ImGuiCol_Separator] = surface1;
    colors[ImGuiCol_SeparatorHovered] = mauve;
    colors[ImGuiCol_SeparatorActive] = mauve;
    colors[ImGuiCol_ResizeGrip] = surface2;
    colors[ImGuiCol_ResizeGripHovered] = mauve;
    colors[ImGuiCol_ResizeGripActive] = mauve;
    colors[ImGuiCol_Tab] = surface0;
    colors[ImGuiCol_TabHovered] = surface2;
    colors[ImGuiCol_TabActive] = surface1;
    colors[ImGuiCol_TabUnfocused] = surface0;
    colors[ImGuiCol_TabUnfocusedActive] = surface1;
    colors[ImGuiCol_DockingPreview] = sapphire;
    colors[ImGuiCol_DockingEmptyBg] = base;
    colors[ImGuiCol_PlotLines] = blue;
    colors[ImGuiCol_PlotLinesHovered] = peach;
    colors[ImGuiCol_PlotHistogram] = teal;
    colors[ImGuiCol_PlotHistogramHovered] = green;
    colors[ImGuiCol_TableHeaderBg] = surface0;
    colors[ImGuiCol_TableBorderStrong] = surface1;
    colors[ImGuiCol_TableBorderLight] = surface0;
    colors[ImGuiCol_TableRowBg] = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);
    colors[ImGuiCol_TableRowBgAlt] = ImVec4(1.0f, 1.0f, 1.0f, 0.06f);
    colors[ImGuiCol_TextSelectedBg] = surface2;
    colors[ImGuiCol_DragDropTarget] = yellow;
    colors[ImGuiCol_NavHighlight] = lavender;
    colors[ImGuiCol_NavWindowingHighlight] = ImVec4(1.0f, 1.0f, 1.0f, 0.7f);
    colors[ImGuiCol_NavWindowingDimBg] = ImVec4(0.8f, 0.8f, 0.8f, 0.2f);
    colors[ImGuiCol_ModalWindowDimBg] = ImVec4(0.0f, 0.0f, 0.0f, 0.35f);
    colors[ImGuiCol_Text] = text;
    colors[ImGuiCol_TextDisabled] = subtext0;

    // Rounded corners for modern look
    style.WindowRounding = 6.0f;
    style.ChildRounding = 6.0f;
    style.FrameRounding = 4.0f;
    style.PopupRounding = 4.0f;
    style.ScrollbarRounding = 9.0f;
    style.GrabRounding = 4.0f;
    style.TabRounding = 4.0f;

    // Padding and spacing
    style.WindowPadding = ImVec2(8.0f, 8.0f);
    style.FramePadding = ImVec2(5.0f, 3.0f);
    style.ItemSpacing = ImVec2(8.0f, 4.0f);
    style.ItemInnerSpacing = ImVec2(4.0f, 4.0f);
    style.IndentSpacing = 21.0f;
    style.ScrollbarSize = 14.0f;
    style.GrabMinSize = 10.0f;

    // Borders
    style.WindowBorderSize = 1.0f;
    style.ChildBorderSize = 1.0f;
    style.PopupBorderSize = 1.0f;
    style.FrameBorderSize = 0.0f;
    style.TabBorderSize = 0.0f;
}

// Setup Catppuccin colormap for ImPlot
void setup_catppuccin_colormap() {
    // Catppuccin Mocha accent colors for plotting
    static const ImVec4 catppuccin_colors[] = {
        ImVec4(Color::CatRed()),      ImVec4(Color::CatPeach()), ImVec4(Color::CatYellow()),   ImVec4(Color::CatGreen()),
        ImVec4(Color::CatTeal()),     ImVec4(Color::CatSky()),   ImVec4(Color::CatSapphire()), ImVec4(Color::CatBlue()),
        ImVec4(Color::CatLavender()), ImVec4(Color::CatMauve()), ImVec4(Color::CatPink()),
    };

    // Add colormap to ImPlot
    ImPlot::AddColormap("Catppuccin", catppuccin_colors, 11);

    // Set as default colormap
    ImPlot::PushColormap("Catppuccin");
}

bool Window::create() {
    // Setup error callback
    glfwSetErrorCallback(glfw_error_callback);

    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW\n";
        return false;
    }

    // Setup OpenGL version
#if defined(__APPLE__)
    // GL 3.2 + GLSL 150 (MacOS)
    const char* glsl_version = "#version 150";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // 3.2+ only
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);           // Required on MacOS
#else
    // GL 3.0 + GLSL 130 (Windows and Linux)
    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
#endif

    // Create window
    _window = glfwCreateWindow(static_cast<int>(_width), static_cast<int>(_height), _name.c_str(), nullptr, nullptr);
    if (!_window) {
        std::cerr << "Failed to create GLFW window\n";
        glfwTerminate();
        return false;
    }
    glfwMakeContextCurrent(_window);
    glfwSwapInterval(0); // Disable vsync

    // Setup context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();

    // Setup ImGui
    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    // Setup Catppuccin Mocha style
    setup_catppuccin_style();

    // Load Inter fonts (Regular and Bold)
    ImGuiIO& io = ImGui::GetIO();
    std::string font_regular_path = core::get_resource_path("fonts/Inter-Regular.ttf");
    std::string font_bold_path = core::get_resource_path("fonts/Inter-Bold.ttf");
    io.Fonts->AddFontFromFileTTF(font_regular_path.c_str(), 16.0f); // Default font (index 0)
    io.Fonts->AddFontFromFileTTF(font_bold_path.c_str(), 16.0f);    // Bold font (index 1)

    // Setup Catppuccin colormap for ImPlot
    setup_catppuccin_colormap();

    // Setup backend
    ImGui_ImplGlfw_InitForOpenGL(_window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    return true;
}

void Window::destroy() {
    if (!_window)
        return;

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    glfwDestroyWindow(_window);
    glfwTerminate();
}

bool Window::should_close() { return glfwWindowShouldClose(_window); }

void Window::begin_frame() {
    // Poll mouse/keyboard events
    glfwPollEvents();

    // Start frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}

void Window::end_frame() {
    // Render
    ImGui::Render();
    int display_w, display_h;
    glfwGetFramebufferSize(_window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(0.117f, 0.117f, 0.172f, 1.0f); // Cat base
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    // Swap buffers
    glfwSwapBuffers(_window);
}

} // namespace gui
