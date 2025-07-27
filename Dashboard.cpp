#include "Dashboard.hpp"
void Dashboard::SetupROS(){
    SOCsub = this->create_subscription<std_msgs::msg::Float64>("SOCTopic", 10, std::bind(&Dashboard::getSOC, this, std::placeholders::_1));
}
void Dashboard::Controller(){

}
void Dashboard::getSOC(const std_msgs::msg::Float64::SharedPtr msg){
    SOC = msg->data;
    std::cout << SOC << std::endl;
    //Need to setup GUI
}

