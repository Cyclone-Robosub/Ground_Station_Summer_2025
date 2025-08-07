#include "DashboardController.hpp"
void DashboardController::SetupROS(){
     callbackBattery = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callbackExecutive = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  auto VoltageOptions = rclcpp::SubscriptionOptions();
  VoltageOptions.callback_group = callbackBattery;

  auto CurrentOptions = rclcpp::SubscriptionOptions();
  CurrentOptions.callback_group = callbackBattery;
 auto CurrentINTOptions = rclcpp::SubscriptionOptions();
 CurrentINTOptions.callback_group = callbackBattery;

 auto WaypointOptions = rclcpp::SubscriptionOptions();
 WaypointOptions.callback_group =  callbackExecutive;
 auto TaskOptions = rclcpp::SubscriptionOptions();
 TaskOptions.callback_group = callbackExecutive;

    SOCsub = this->create_subscription<std_msgs::msg::Float64>("SOCTopic", 10, std::bind(&DashboardController::getSOC, this, std::placeholders::_1), CurrentOptions);
     SOCINTsub = this->create_subscription<std_msgs::msg::Bool>("SOCIntTopic", 10, std::bind(&DashboardController::getSOCINT, this, std::placeholders::_1), CurrentINTOptions);
     WaypointSub = this->create_subscription<std_msgs::msg::Float32MultiArray>("waypoint_topic", 10, std::bind(&DashboardController::getWaypoint, this, std::placeholders::_1), WaypointOptions);
     CurrentTaskSub =  this->create_subscription<std_msgs::msg::String>("CurrentTaskTopic", 10, std::bind(&DashboardController::getCurrentTask, this, std::placeholders::_1), TaskOptions);
    //manipulation
    //Vision
    
}
void DashboardController::Controller(){
    while(!isControllerShutdown.load()){
        //Robot not Running
    if(CurrentTask.load(std::memory_order_acquire) != nullptr){
        isRobotRunning = true;
    }
    //Flash Blue for Robot not Running.
    while(isRobotRunning){
        //If SOC INT is true -> push it out Red on main light.

        //if Everything is good push it out all green.
    
    }
    }

    //
}
void DashboardController::getSOC(const std_msgs::msg::Float64::SharedPtr msg){
    SOC.store(msg->data, std::memory_order_release);
   // std::cout << SOC << std::endl;
    //Need to setup GUI
}
void DashboardController::getSOCINT(const std_msgs::msg::Bool::SharedPtr msg){
    SOCint.store(msg->data, std::memory_order_release);
}

void DashboardController::getWaypoint(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    std::array<float, 6> arr;
    std::copy_n(msg->data.begin(), 6, arr.begin());
    CurrentWaypoint.store(std::make_shared<std::array<float, 6>>(arr), std::memory_order_release);
}

void DashboardController::getCurrentTask(const std_msgs::msg::String::SharedPtr msg){
    CurrentTask.store(std::make_shared<std::string>(msg->data), std::memory_order_release);
}


void DashboardController::Shutdown(){
    isControllerShutdown = true;
}
