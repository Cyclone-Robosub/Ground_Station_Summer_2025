# Ground Station - Beta
This is a DearImGUI implementation that allows for Ethernet or Wifi connection to the AUV for real-time data visualization and control of the AUV during the 2024-2025 season. For now, control of the AUV is limited to the ability to change the PID constants during or before the run. 
## Running the Program
Make sure that you have sourced ROS
```bash
source /opt/ros/jazzy/setup.bash
```
Build the project
```bash
colcon build
```
Run the excutable
```bash
cd build/dashboard_project
./DashboardExecutable
```
Or alternatively 
```
build/dashboard_project/DashboardExecutable
```
## Resources
- [Interactable Demo Site](https://pthom.github.io/imgui_manual_online/manual/imgui_manual.html)
