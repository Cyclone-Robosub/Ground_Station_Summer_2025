#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <iostream>
#include <thread>
#include <string>
#include <atomic>

class Dashboard : public rclcpp::Node {
public:
  Dashboard() : Node("DashboardNode") {
    SetupROS();
    // Controller();
  }
  std::atomic<float> SOC;
  std::atomic<bool> SOCint{false};
  std::atomic<std::shared_ptr<std::string>> CurrentTask;
  std::atomic<int> ManipulationCode;
private:
  void SetupROS();
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr SOCsub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr SOCINTsub;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr WaypointSub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr CurrentTaskSub;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr ManipulationSub;

  void getSOC(const std_msgs::msg::Float64::SharedPtr msg);
  void getSOCINT(const std_msgs::msg::Bool::SharedPtr msg);
  void getWaypoint(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void getCurrentTask(const std_msgs::msg::String::SharedPtr msg);
  void getMainipulation(const std_msgs::msg::Int64::SharedPtr msg);
  void Controller();
  bool isRobotRunning;
};