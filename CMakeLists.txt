cmake_minimum_required(VERSION 3.20)
project(dashboard_project)

set(IMGUI_DIR imgui-master)
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
    Dashboard.hpp
    Dashboard.cpp
    DashboardGUI.cpp
    components/StatusIndicators.hpp
    components/StatusIndicators.cpp
    components/BatteryMonitor.hpp
    components/BatteryMonitor.cpp
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_demo.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
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






