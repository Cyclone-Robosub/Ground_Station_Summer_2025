#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <iostream>
#include <thread>
#include <string>
#include <atomic>
#include "StructComponents.hpp"
#include "../crs_common/position/position.hpp"
/// @todo : multithreadexecutor for multiple subscribers.
///         Subscription Options
///         
class DashboardController : public rclcpp::Node {
public:
  DashboardController(std::shared_ptr<StructofComponents> givenComponentStruct) : Node("DashboardNode"), ComponentStruct(givenComponentStruct) {
    SetupROS();
  }
  void Controller();
  void Shutdown();
private:
  void SetupROS();
  std::shared_ptr<StructofComponents> ComponentStruct;

  rclcpp::CallbackGroup::SharedPtr callbackBattery;
  rclcpp::CallbackGroup::SharedPtr callbackExecutive;
  rclcpp::CallbackGroup::SharedPtr callbackThruster;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr SOCsub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr SOCINTsub;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr Voltsub;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr WaypointSub;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr PositionSub;

  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr PWMSub;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr CurrentTaskSub;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr ManipulationSub;

  void getSOC(const std_msgs::msg::Float64::SharedPtr msg);
  void getSOCINT(const std_msgs::msg::Bool::SharedPtr msg);
  void getWaypoint(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void getPosition(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void getCurrentTask(const std_msgs::msg::String::SharedPtr msg);
  void getVolt(const std_msgs::msg::Float64::SharedPtr msg);
  void getMainipulation(const std_msgs::msg::Int64::SharedPtr msg);
  void getPWM(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
  bool isRobotRunning;
  std::atomic<bool> isControllerShutdown{false};
  bool isSoftwareKS;
  bool isHardwareKS;
};
