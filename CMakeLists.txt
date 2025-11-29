cmake_minimum_required(VERSION 3.14)
project(MSCKFPlayground LANGUAGES CXX C)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include(FetchContent)

# Setup OpenGL
cmake_policy(SET CMP0072 NEW) # Pefer GLVND over legacy GL libraries
find_package(OpenGL REQUIRED)

# Setup GLFW
#FetchContent_Declare(
#    glfw
#    GIT_REPOSITORY "https://github.com/glfw/glfw"
#    GIT_TAG "3.3.8"
#    GIT_PROGRESS TRUE
#    GIT_SHALLOW TRUE
#)
#FetchContent_MakeAvailable(glfw)
find_package(glfw3 REQUIRED)

# Setup ImGui
#FetchContent_Declare(
#    imgui
#    GIT_REPOSITORY "https://github.com/ocornut/imgui"
#    GIT_TAG "v1.92.4"
#    GIT_PROGRESS TRUE
#    GIT_SHALLOW TRUE
#)
#FetchContent_MakeAvailable(imgui)
set(imgui_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/build/_deps/imgui-src)
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
set(implot_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/build/_deps/implot-src)
set(IMPLOT_SOURCE
    ${implot_SOURCE_DIR}/implot.cpp
    ${implot_SOURCE_DIR}/implot_demo.cpp
    ${implot_SOURCE_DIR}/implot_items.cpp
)
add_library(implot STATIC ${IMPLOT_SOURCE})
target_include_directories(implot PUBLIC ${implot_SOURCE_DIR})
target_link_libraries(implot PUBLIC imgui)

# Setup Eigen
#set(FETCHCONTENT_QUIET OFF)
#FetchContent_Declare(
#    Eigen
#    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
#    GIT_TAG 3.4.0
#    GIT_SHALLOW TRUE
#)
#FetchContent_MakeAvailable(Eigen)
find_package(Eigen3)

# Setup executable
set(exe_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(EXE_SOURCE
    ${exe_SOURCE_DIR}/main.cpp

    # GUI
    ${exe_SOURCE_DIR}/gui/app.cpp
    ${exe_SOURCE_DIR}/gui/window.cpp
)
add_executable(msckf_playground ${EXE_SOURCE})
target_include_directories(msckf_playground PRIVATE ${exe_SOURCE_DIR})
target_link_libraries(msckf_playground PRIVATE implot Eigen3::Eigen)

# Silence OpenGL deprecation warnings on macOS
if(APPLE)
    target_compile_definitions(msckf_playground PRIVATE GL_SILENCE_DEPRECATION)
endif()
