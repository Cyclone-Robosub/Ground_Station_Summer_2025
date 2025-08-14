#!/bin/bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
cd build/dashboard_project

export ROS_DOMAIN_ID=4
./DashboardExecutable
