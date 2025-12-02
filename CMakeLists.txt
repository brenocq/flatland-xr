cmake_minimum_required(VERSION 3.14)
project(flatland-xr LANGUAGES CXX C)

# Options
option(FLATLAND_XR_BUILD_TESTS "Build unit tests" ON)
option(FLATLAND_XR_STRICT_WARNINGS "Enable strict compiler warnings and treat as errors" OFF)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include(FetchContent)

# Setup OpenGL
# cmake-format: off
cmake_policy(SET CMP0072 NEW) # Pefer GLVND over legacy GL libraries
# cmake-format: on
find_package(OpenGL REQUIRED)

# Setup Eigen (disable tests, docs, etc.)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    eigen
    GIT_REPOSITORY "https://gitlab.com/libeigen/eigen.git"
    GIT_TAG "3.4.0"
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(eigen)

# Setup GLFW (disable tests, examples, docs)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
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
target_include_directories(imgui PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src;${imgui_SOURCE_DIR};${imgui_SOURCE_DIR}/backends/")
target_compile_definitions(imgui PUBLIC IMGUI_USER_CONFIG="thirdparty/imconfig_flatland.h")
target_link_libraries(imgui PUBLIC glfw OpenGL::GL)

# Setup ImPlot
FetchContent_Declare(
    implot
    GIT_REPOSITORY "https://github.com/epezent/implot"
    GIT_TAG "8a9d793e47ca8de526c401b6bc788e1ff2967a31"
    GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(implot)
set(IMPLOT_SOURCE ${implot_SOURCE_DIR}/implot.cpp ${implot_SOURCE_DIR}/implot_demo.cpp ${implot_SOURCE_DIR}/implot_items.cpp)
add_library(implot STATIC ${IMPLOT_SOURCE})
target_include_directories(implot PUBLIC ${implot_SOURCE_DIR})
target_link_libraries(implot PUBLIC imgui)

# ============================================================================
# Module Libraries
# ============================================================================
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

# Function to apply strict warnings to a target
function(flatland_xr_target_strict_warnings target)
    if(FLATLAND_XR_STRICT_WARNINGS)
        if(MSVC)
            target_compile_options(${target} PRIVATE /W4 /WX)
        else()
            target_compile_options(${target} PRIVATE -Wall -Wextra -Werror)
        endif()
    endif()
endfunction()

# MSVC-specific settings
if(MSVC)
    # Enable M_PI and other math constants from <cmath>
    add_compile_definitions(_USE_MATH_DEFINES)
    # Disable warnings from external headers (Eigen)
    add_compile_options(/external:anglebrackets /external:W0)
else()
    # GCC/Clang: Treat Eigen as system headers to suppress warnings This is important when FLATLAND_XR_STRICT_WARNINGS is enabled
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        include_directories(SYSTEM ${CMAKE_BINARY_DIR}/_deps/eigen-src)
    endif()
endif()

# Core library
add_library(flatland-core STATIC ${SRC_DIR}/core/geometry.cpp ${SRC_DIR}/core/trajectory2d.cpp)
target_include_directories(flatland-core PUBLIC ${SRC_DIR})
target_link_libraries(flatland-core PUBLIC Eigen3::Eigen)
flatland_xr_target_strict_warnings(flatland-core)

# Sensors library
add_library(flatland-sensors STATIC ${SRC_DIR}/sensors/camera2d.cpp ${SRC_DIR}/sensors/imu2d.cpp)
target_include_directories(flatland-sensors PUBLIC ${SRC_DIR})
target_link_libraries(flatland-sensors PUBLIC flatland-core)
flatland_xr_target_strict_warnings(flatland-sensors)

# World library
add_library(flatland-world STATIC ${SRC_DIR}/world/world.cpp)
target_include_directories(flatland-world PUBLIC ${SRC_DIR})
target_link_libraries(flatland-world PUBLIC flatland-core)
flatland_xr_target_strict_warnings(flatland-world)

# Simulation library
add_library(flatland-simulation STATIC ${SRC_DIR}/simulation/simulation.cpp)
target_include_directories(flatland-simulation PUBLIC ${SRC_DIR})
target_link_libraries(flatland-simulation PUBLIC flatland-core flatland-sensors flatland-world)
flatland_xr_target_strict_warnings(flatland-simulation)

# Estimation library
add_library(flatland-estimation STATIC ${SRC_DIR}/estimation/imu_integrator.cpp)
target_include_directories(flatland-estimation PUBLIC ${SRC_DIR})
target_link_libraries(flatland-estimation PUBLIC flatland-core flatland-sensors)
flatland_xr_target_strict_warnings(flatland-estimation)

# GUI library (depends on ImGui/ImPlot)
add_library(
    flatland-gui STATIC
    ${SRC_DIR}/gui/app.cpp
    ${SRC_DIR}/gui/color.cpp
    ${SRC_DIR}/gui/plot.cpp
    ${SRC_DIR}/gui/ui_state.cpp
    ${SRC_DIR}/gui/window.cpp
    ${SRC_DIR}/gui/widgets/text.cpp
    ${SRC_DIR}/gui/panels/config_panel.cpp
    ${SRC_DIR}/gui/panels/error_metrics_panel.cpp
    ${SRC_DIR}/gui/panels/measurements_panel.cpp
    ${SRC_DIR}/gui/panels/perception_output_panel.cpp
    ${SRC_DIR}/gui/panels/world_editor_panel.cpp
)
target_include_directories(flatland-gui PUBLIC ${SRC_DIR})
target_link_libraries(flatland-gui PUBLIC flatland-simulation flatland-estimation implot)
flatland_xr_target_strict_warnings(flatland-gui)

# ============================================================================
# Main Executable
# ============================================================================
add_executable(flatland-xr ${SRC_DIR}/main.cpp)
target_link_libraries(flatland-xr PRIVATE flatland-gui)
flatland_xr_target_strict_warnings(flatland-xr)

# Silence OpenGL deprecation warnings on macOS
if(APPLE)
    target_compile_definitions(flatland-xr PRIVATE GL_SILENCE_DEPRECATION)
endif()

# ============================================================================
# Testing
# ============================================================================
if(FLATLAND_XR_BUILD_TESTS)
    enable_testing()

    # Setup Google Test
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY "https://github.com/google/googletest"
        GIT_TAG "v1.14.0"
        GIT_SHALLOW TRUE
    )
    # Prevent overriding the parent project's compiler/linker settings on Windows
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    # Test sources
    set(TEST_SOURCE
        tests/core/test_geometry.cpp
        tests/core/test_trajectory2d.cpp
        tests/core/test_types.cpp
        tests/sensors/test_camera2d.cpp
        tests/sensors/test_imu2d.cpp
        tests/simulation/test_simulation.cpp
        tests/world/test_world.cpp
        tests/estimation/test_imu_integrator.cpp
    )
    add_executable(flatland-tests ${TEST_SOURCE})
    target_link_libraries(
        flatland-tests PRIVATE flatland-core flatland-sensors flatland-world flatland-simulation flatland-estimation GTest::gtest_main
    )
    flatland_xr_target_strict_warnings(flatland-tests)

    include(GoogleTest)
    gtest_discover_tests(flatland-tests)
endif()

# Copy resources to build directory for development
add_custom_command(
    TARGET flatland-xr
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/resources ${CMAKE_BINARY_DIR}/resources
    COMMENT "Copying resources to build directory"
)

# Configure source directory path for resource loading
target_compile_definitions(flatland-xr PRIVATE FLATLAND_XR_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
