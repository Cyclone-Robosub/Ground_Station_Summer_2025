cmake_minimum_required(VERSION 3.20)
project(cyclone_dashboard)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(EXPORT_COMPILE_COMMANDS ON)

# -------------------------------------------------------------------------------------------------
# ROS 2 Dependencies
# -------------------------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Threads REQUIRED)

# -------------------------------------------------------------------------------------------------
# Third-Party Dependencies (now in the 'include' subdirectory)
# -------------------------------------------------------------------------------------------------
add_subdirectory(include/glfw)
find_package(OpenGL REQUIRED)

# --- ImGui Library ---
# The source files are now located under the `include/imgui` directory.
add_library(imgui STATIC
    include/imgui/imgui.cpp
    include/imgui/imgui_demo.cpp
    include/imgui/imgui_draw.cpp
    include/imgui/imgui_tables.cpp
    include/imgui/imgui_widgets.cpp
    include/imgui/backends/imgui_impl_glfw.cpp
    include/imgui/backends/imgui_impl_opengl3.cpp
)
# The public include directories have also been updated to the new path.
target_include_directories(imgui PUBLIC
    ${PROJECT_SOURCE_DIR}/include/imgui
    ${PROJECT_SOURCE_DIR}/include/imgui/backends
)
target_compile_definitions(imgui PRIVATE IMGUI_IMPL_OPENGL_LOADER_GLAD)
target_link_libraries(imgui PUBLIC glfw OpenGL::GL)

# --- ImPlot Library ---
# The source files are now located under the `include/implot` directory.
add_library(implot STATIC
    include/implot/implot.cpp
    include/implot/implot_items.cpp
    include/implot/implot_demo.cpp
)
# The public include directories have also been updated to the new path.
target_include_directories(implot PUBLIC
    ${PROJECT_SOURCE_DIR}/include/implot
    ${PROJECT_SOURCE_DIR}/include/imgui
)
target_link_libraries(implot PUBLIC imgui)

# --- ImPlot3D Library ---
# The source files are now located under the `include/implot3d` directory.
add_library(implot3d STATIC
    include/implot3d/implot3d.cpp
    include/implot3d/implot3d_items.cpp
    include/implot3d/implot3d_demo.cpp
    include/implot3d/implot3d_meshes.cpp
)
# The public include directories have also been updated to the new path.
target_include_directories(implot3d PUBLIC
    ${PROJECT_SOURCE_DIR}/include/implot3d
    ${PROJECT_SOURCE_DIR}/include/implot
    ${PROJECT_SOURCE_DIR}/include/imgui
)
target_link_libraries(implot3d PUBLIC implot)

# -------------------------------------------------------------------------------------------------
# Main Executable
# -------------------------------------------------------------------------------------------------
add_executable(DashboardExecutable
    main.cpp
    DashboardController.cpp
    DashboardGUI.cpp
    components/StatusIndicators.cpp
    components/BatteryMonitor.cpp
    components/MessageLogger.cpp
    components/ConfigurationPanel.cpp
    components/MultiAxisPlotting.cpp
    components/SingleAxisPlotting.cpp
    StructComponents.hpp
    ../crs_common/position/position.cpp
)
# Use ament_target_dependencies to handle ROS 2 dependencies correctly
ament_target_dependencies(DashboardExecutable
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    diagnostic_msgs
    std_srvs
)

# Link all *other* dependencies to the executable
target_link_libraries(DashboardExecutable
    # Our custom libraries
    implot3d
    implot
    imgui

    # System libraries
    ${GLFW_LIBRARIES}
    OpenGL::GL
    Threads::Threads
)

# -------------------------------------------------------------------------------------------------
# Installation and Export
# -------------------------------------------------------------------------------------------------
install(TARGETS DashboardExecutable DESTINATION lib/${PROJECT_NAME})
ament_package()

