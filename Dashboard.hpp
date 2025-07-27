#include <chrono>
#include <iostream>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class Dashboard : public rclcpp::Node
{
public:
  Dashboard() : Node("DashboardNode"){
    SetupROS();
    Controller();
  }
private:
  void SetupROS();
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr SOCsub;
  void getSOC(const std_msgs::msg::Float64::SharedPtr msg);
  float SOC;
  void Controller();
};