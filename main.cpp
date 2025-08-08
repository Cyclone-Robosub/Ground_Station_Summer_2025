#include "rclcpp/rclcpp.hpp"
#include "DashboardGUI.hpp"
#include "StructComponents.hpp"
#include <thread>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);


  std::shared_ptr<StructofComponents> ComponentStruct = std::make_shared<StructofComponents>();
  std::shared_ptr<DashboardController> DashboardPtr = std::make_shared<DashboardController>(ComponentStruct);
  std::jthread ROSThread([DashboardPtr]() { rclcpp::spin(DashboardPtr); });
  DashboardGUI DashboardGUIObject = DashboardGUI(DashboardPtr, ComponentStruct);
  DashboardGUIObject.Startup();
  rclcpp::shutdown();
}