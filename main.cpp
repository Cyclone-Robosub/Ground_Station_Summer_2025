
#include <ctime>
#include <chrono>
#include <iostream>
#include "Dashboard.hpp"
#include <rclcpp/rclcpp.hpp>
std::shared_ptr<Dashboard> Startup() {
  return std::make_shared<Dashboard>();
}


int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Dashboard>());
  return 0;
}
