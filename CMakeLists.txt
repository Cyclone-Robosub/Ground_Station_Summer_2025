cmake_minimum_required(VERSION 3.20)
project(dashboard_project)

set(IMGUI_DIR imgui-master)
set(IMPLOT_DIR implot)
set(IMPLOT3D_DIR implot3d)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(EXPORT_COMPILE_COMMANDS ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Find and link GLFW
find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Threads REQUIRED)
# Add include directories
include_directories(
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
    ${IMPLOT_DIR}
    ${IMPLOT3D_DIR}
)


include_directories(${GLFW_INCLUDE_DIRS})
link_directories(${GLFW_LIBRARY_DIRS})
add_definitions(${GLFW_CFLAGS_OTHER})
# Platform-specific settings
if(APPLE)
    message(STATUS "Detected platform: macOS")
    set(PLATFORM_LIBS
        ${GLFW_LIBRARIES}
        "-framework OpenGL"
        "-framework Cocoa"
        "-framework IOKit"
        "-framework CoreVideo"
    )
elseif(UNIX AND NOT APPLE)
    message(STATUS "Detected platform: Linux")
    set(PLATFORM_LIBS
        ${GLFW_LIBRARIES}
        GL
    )
elseif(WIN32)
    message(STATUS "Detected platform: Windows")
    set(PLATFORM_LIBS
        glfw3
        gdi32
        opengl32
        imm32
    )
endif()


add_executable(DashboardExecutable
    main.cpp
    DashboardController.hpp
    DashboardController.cpp
    DashboardGUI.cpp
    components/StatusIndicators.hpp
    components/StatusIndicators.cpp
    components/BatteryMonitor.hpp
    components/BatteryMonitor.cpp
    components/MessageLogger.hpp
    components/MessageLogger.cpp
    components/ConfigurationPanel.hpp
    components/ConfigurationPanel.cpp
    components/RobotPlotting.hpp
    components/RobotPlotting.cpp
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_demo.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
    ${IMPLOT_DIR}/implot.cpp
    ${IMPLOT_DIR}/implot_demo.cpp
    ${IMPLOT_DIR}/implot_internal.h
    ${IMPLOT_DIR}/implot_items.cpp
    ${IMPLOT_DIR}/implot.h
    ${IMPLOT3D_DIR}/implot3d.cpp
    ${IMPLOT3D_DIR}/implot3d_demo.cpp
    ${IMPLOT3D_DIR}/implot3d_internal.h
    ${IMPLOT3D_DIR}/implot3d_items.cpp
    ${IMPLOT3D_DIR}/implot3d.h

)

ament_target_dependencies(DashboardExecutable
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  diagnostic_msgs
  std_srvs
)

# Correct target name here
target_link_libraries(DashboardExecutable ${PLATFORM_LIBS} Threads::Threads)

install(TARGETS DashboardExecutable DESTINATION lib/${PROJECT_NAME})

ament_package()






