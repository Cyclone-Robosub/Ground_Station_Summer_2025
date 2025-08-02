#include "Dashboard.hpp"
void Dashboard::SetupROS(){
    SOCsub = this->create_subscription<std_msgs::msg::Float64>("SOCTopic", 10, std::bind(&Dashboard::getSOC, this, std::placeholders::_1));
     SOCINTsub = this->create_subscription<std_msgs::msg::Bool>("SOCINTTopic", 10, std::bind(&Dashboard::getSOCINT, this, std::placeholders::_1));

}
void Dashboard::Controller(){
    while(true){
        //Robot not Running
    while(isRobotRunning){
    //Check SOCint, then do something if true
    }
    }
}
void Dashboard::getSOC(const std_msgs::msg::Float64::SharedPtr msg){
    SOC = msg->data;
    std::cout << SOC << std::endl;
    //Need to setup GUI
}
void Dashboard::getSOCINT(const std_msgs::msg::Bool::SharedPtr msg){
    SOCint = msg->data;
}
