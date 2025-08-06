#include "Dashboard.hpp"
void Dashboard::SetupROS(){
     callbackBattery = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto VoltageOptions = rclcpp::SubscriptionOptions();
  VoltageOptions.callback_group = callbackBattery;

  auto CurrentOptions = rclcpp::SubscriptionOptions();
  CurrentOptions.callback_group = callbackBattery;
 auto CurrentINTOptions = rclcpp::SubscriptionOptions();
 CurrentINTOptions.callback_group = callbackBattery;
    SOCsub = this->create_subscription<std_msgs::msg::Float64>("SOCTopic", 10, std::bind(&Dashboard::getSOC, this, std::placeholders::_1), CurrentOptions);
     SOCINTsub = this->create_subscription<std_msgs::msg::Bool>("SOCIntTopic", 10, std::bind(&Dashboard::getSOCINT, this, std::placeholders::_1), CurrentINTOptions);
    //manipulation
    //Vision
    
}
void Dashboard::Controller(){
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
void Dashboard::getSOC(const std_msgs::msg::Float64::SharedPtr msg){
    SOC.store(msg->data, std::memory_order_release);
   // std::cout << SOC << std::endl;
    //Need to setup GUI
}
void Dashboard::getSOCINT(const std_msgs::msg::Bool::SharedPtr msg){
    SOCint.store(msg->data, std::memory_order_release);
}
void Dashboard::Shutdown(){
    isControllerShutdown = true;
}
