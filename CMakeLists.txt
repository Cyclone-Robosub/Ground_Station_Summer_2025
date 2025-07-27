cmake_minimum_required(VERSION 3.20)
project(dashboard_project)


set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(EXPORT_COMPILE_COMMANDS ON)
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


add_executable(DashboardExecutable
    main.cpp
    Dashboard.hpp
    Dashboard.cpp
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
install(TARGETS DashboardExecutable DESTINATION lib/${PROJECT_NAME})

ament_package()