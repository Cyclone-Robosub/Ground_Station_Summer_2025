#include <chrono>
#include <iostream>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"

class Dashboard : public rclcpp::Node
{
public:
  Dashboard() : Node("DashboardNode"){
    SetupROS();
    //Controller();
  }
  std::atomic<float> SOC;
  std::atomic<bool> SOCint{false};
private:
  void SetupROS();
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr SOCsub;
   rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr SOCINTsub;
  void getSOC(const std_msgs::msg::Float64::SharedPtr msg);
  void getSOCINT(const std_msgs::msg::Bool::SharedPtr msg);
  void Controller();
  bool isRobotRunning;

};