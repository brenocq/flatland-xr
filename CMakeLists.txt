cmake_minimum_required(VERSION 3.14)
project(flatland-xr LANGUAGES CXX C)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include(FetchContent)

# Setup OpenGL
# cmake-format: off
cmake_policy(SET CMP0072 NEW) # Pefer GLVND over legacy GL libraries
# cmake-format: on
find_package(OpenGL REQUIRED)

# Setup Eigen
FetchContent_Declare(
    eigen
    GIT_REPOSITORY "https://gitlab.com/libeigen/eigen.git"
    GIT_TAG "3.4.0"
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(eigen)

# Setup GLFW
FetchContent_Declare(
    glfw
    GIT_REPOSITORY "https://github.com/glfw/glfw"
    GIT_TAG "3.3.8"
    GIT_PROGRESS TRUE
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(glfw)

# Setup ImGui
FetchContent_Declare(
    imgui
    GIT_REPOSITORY "https://github.com/ocornut/imgui"
    GIT_TAG "v1.92.5-docking"
    GIT_PROGRESS TRUE
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(imgui)
set(IMGUI_SOURCE
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)
add_library(imgui STATIC ${IMGUI_SOURCE})
target_include_directories(imgui PUBLIC "${imgui_SOURCE_DIR};${imgui_SOURCE_DIR}/backends/")
target_link_libraries(imgui PUBLIC glfw OpenGL::GL)

# Setup ImPlot
FetchContent_Declare(
    implot
    GIT_REPOSITORY "https://github.com/epezent/implot"
    GIT_TAG "8a9d793e47ca8de526c401b6bc788e1ff2967a31"
    GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(implot)
set(IMPLOT_SOURCE
    ${implot_SOURCE_DIR}/implot.cpp
    ${implot_SOURCE_DIR}/implot_demo.cpp
    ${implot_SOURCE_DIR}/implot_items.cpp
)
add_library(implot STATIC ${IMPLOT_SOURCE})
target_include_directories(implot PUBLIC ${implot_SOURCE_DIR})
target_link_libraries(implot PUBLIC imgui)

# Setup executable
set(exe_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(EXE_SOURCE
    ${exe_SOURCE_DIR}/main.cpp
    # Core
    ${exe_SOURCE_DIR}/core/geometry.cpp
    ${exe_SOURCE_DIR}/core/trajectory2d.cpp
    # Sensors
    ${exe_SOURCE_DIR}/sensors/camera2d.cpp
    ${exe_SOURCE_DIR}/sensors/imu2d.cpp
    # Simulation
    ${exe_SOURCE_DIR}/simulation/simulation.cpp
    # World
    ${exe_SOURCE_DIR}/world/world.cpp
    # Estimation
    ${exe_SOURCE_DIR}/estimation/imu_integrator.cpp
    # GUI
    ${exe_SOURCE_DIR}/gui/app.cpp
    ${exe_SOURCE_DIR}/gui/color.cpp
    ${exe_SOURCE_DIR}/gui/plot.cpp
    ${exe_SOURCE_DIR}/gui/window.cpp
    # GUI Panels
    ${exe_SOURCE_DIR}/gui/panels/config_panel.cpp
    ${exe_SOURCE_DIR}/gui/panels/error_metrics_panel.cpp
    ${exe_SOURCE_DIR}/gui/panels/measurements_panel.cpp
    ${exe_SOURCE_DIR}/gui/panels/perception_output_panel.cpp
    ${exe_SOURCE_DIR}/gui/panels/world_editor_panel.cpp
)
add_executable(flatland-xr ${EXE_SOURCE})
target_include_directories(flatland-xr PRIVATE ${exe_SOURCE_DIR} ${exe_SOURCE_DIR}/core ${exe_SOURCE_DIR}/sensors ${exe_SOURCE_DIR}/simulation ${exe_SOURCE_DIR}/world ${eigen_SOURCE_DIR})
target_link_libraries(flatland-xr PRIVATE implot Eigen3::Eigen)

# Silence OpenGL deprecation warnings on macOS
if(APPLE)
    target_compile_definitions(flatland-xr PRIVATE GL_SILENCE_DEPRECATION)
endif()
