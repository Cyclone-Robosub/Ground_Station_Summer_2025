cmake_minimum_required(VERSION 3.20)
project(dashboard_project)

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
# Third-Party Dependencies (not in separate CMake projects)
# -------------------------------------------------------------------------------------------------
find_package(glfw3 3.3 REQUIRED)
find_package(OpenGL REQUIRED)

# --- ImGui Library ---
add_library(imgui STATIC
    imgui/imgui.cpp
    imgui/imgui_demo.cpp
    imgui/imgui_draw.cpp
    imgui/imgui_tables.cpp
    imgui/imgui_widgets.cpp
    imgui/backends/imgui_impl_glfw.cpp
    imgui/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui PUBLIC
    ${PROJECT_SOURCE_DIR}/imgui
    ${PROJECT_SOURCE_DIR}/imgui/backends
)
target_compile_definitions(imgui PRIVATE IMGUI_IMPL_OPENGL_LOADER_GLAD)
target_link_libraries(imgui PUBLIC glfw OpenGL::GL)

# --- ImPlot Library ---
add_library(implot STATIC
    implot/implot.cpp
    implot/implot_items.cpp
    implot/implot_demo.cpp
)
target_include_directories(implot PUBLIC
    ${PROJECT_SOURCE_DIR}/implot
    ${PROJECT_SOURCE_DIR}/imgui
)
target_link_libraries(implot PUBLIC imgui)

# --- ImPlot3D Library ---
add_library(implot3d STATIC
    implot3d/implot3d.cpp
    implot3d/implot3d_items.cpp
    implot3d/implot3d_demo.cpp
    implot3d/implot3d_meshes.cpp
)
target_include_directories(implot3d PUBLIC
    ${PROJECT_SOURCE_DIR}/implot3d
    ${PROJECT_SOURCE_DIR}/implot
    ${PROJECT_SOURCE_DIR}/imgui
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
    components/RobotPlotting.cpp
    StructComponents.hpp
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