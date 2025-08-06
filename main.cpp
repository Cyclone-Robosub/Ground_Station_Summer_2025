#include "rclcpp/rclcpp.hpp"
#include "DashboardGUI.hpp"
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<DashboardController> DashboardPtr = std::make_shared<DashboardController>();
  std::thread ROSThread([DashboardPtr]() { rclcpp::spin(DashboardPtr); });
  ROSThread.detach();
  DashboardGUI DashboardGUIObject = DashboardGUI(DashboardPtr);
  DashboardGUIObject.Startup();
  rclcpp::shutdown();
}