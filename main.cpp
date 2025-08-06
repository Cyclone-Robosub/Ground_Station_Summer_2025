#include "rclcpp/rclcpp.hpp"
#include "DashboardGUI.hpp"
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<Dashboard> DashboardPtr = std::make_shared<Dashboard>();
  std::thread ROSThread([DashboardPtr]() { rclcpp::spin(DashboardPtr); });
  ROSThread.detach();
  DashboardGUI DashboardGUIObject = DashboardGUI(DashboardPtr);
  DashboardGUIObject.Startup();
  rclcpp::shutdown();
}