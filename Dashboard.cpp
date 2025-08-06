#include "Dashboard.hpp"
void Dashboard::SetupROS(){
    
    SOCsub = this->create_subscription<std_msgs::msg::Float64>("SOCTopic", 10, std::bind(&Dashboard::getSOC, this, std::placeholders::_1));
     SOCINTsub = this->create_subscription<std_msgs::msg::Bool>("SOCINTTopic", 10, std::bind(&Dashboard::getSOCINT, this, std::placeholders::_1));
    //manipulation
    //Vision
    
}
void Dashboard::Controller(){
    while(true){
        //Robot not Running
    if(CurrentTask.load(std::memory_order_acquire) != nullptr){
        isRobotRunning = true;
    }
    //Flash Blue for Robot not Running.
    while(isRobotRunning){
        //If SOC INT is true -> push it out Red on main light.

        //if Everything is good push it out all green.
        //I suggest having a color class of all lights that have string member var to specify the type.
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
