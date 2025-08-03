#include "rclcpp/rclcpp.hpp"
#include "Dashboard.hpp"
#include "DashboardGUI.cpp"
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<Dashboard> node = std::make_shared<Dashboard>();
  std::thread ROSThread([node]() { rclcpp::spin(node); });
  ROSThread.detach();
  std::thread GUIThread(Startup);
  GUIThread.detach();
}