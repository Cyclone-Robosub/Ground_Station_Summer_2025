#include "DashboardController.hpp"
#include <algorithm>
void DashboardController::SetupROS()
{
    callbackBattery = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callbackExecutive = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    callbackThruster = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callbackKillSwitch = this->create_callback_group
    (rclcpp::CallbackGroupType::MutuallyExclusive);

    auto VoltageOptions = rclcpp::SubscriptionOptions();
    VoltageOptions.callback_group = callbackBattery;

    auto CurrentOptions = rclcpp::SubscriptionOptions();
    CurrentOptions.callback_group = callbackBattery;
    auto CurrentINTOptions = rclcpp::SubscriptionOptions();
    CurrentINTOptions.callback_group = callbackBattery;
    auto VoltOptions = rclcpp::SubscriptionOptions();
    VoltOptions.callback_group = callbackBattery;

    auto WaypointOptions = rclcpp::SubscriptionOptions();
    WaypointOptions.callback_group = callbackExecutive;
    auto TaskOptions = rclcpp::SubscriptionOptions();
    TaskOptions.callback_group = callbackExecutive;
    auto PositionOptions = rclcpp::SubscriptionOptions();
    PositionOptions.callback_group = callbackExecutive;

    auto PWMOptions = rclcpp::SubscriptionOptions();
    PWMOptions.callback_group = callbackThruster;

    auto SoftwareKSOptions = rclcpp::SubscriptionOptions();
    SoftwareKSOptions.callback_group = callbackKillSwitch;

   Voltsub = this->create_subscription<std_msgs::msg::Float64>("voltageReadingTopic", 10, std::bind(&DashboardController::getVolt, this, std::placeholders::_1), VoltOptions);
    SOCsub = this->create_subscription<std_msgs::msg::Float64>("SOCTopic", 10, std::bind(&DashboardController::getSOC, this, std::placeholders::_1), CurrentOptions);
    SOCINTsub = this->create_subscription<std_msgs::msg::Bool>("SOCIntTopic", 10, std::bind(&DashboardController::getSOCINT, this, std::placeholders::_1), CurrentINTOptions);
    WaypointSub = this->create_subscription<std_msgs::msg::Float32MultiArray>("waypoint_topic", 10, std::bind(&DashboardController::getWaypoint, this, std::placeholders::_1), WaypointOptions);
    CurrentTaskSub = this->create_subscription<std_msgs::msg::String>("CurrentTaskTopic", 10, std::bind(&DashboardController::getCurrentTask, this, std::placeholders::_1), TaskOptions);
    PositionSub = 
     this->create_subscription<std_msgs::msg::Float32MultiArray>("position_topic", 10, std::bind(&DashboardController::getPosition, this, std::placeholders::_1), PositionOptions);
     PWMSub = 
 	this->create_subscription<std_msgs::msg::Int32MultiArray>("sent_pwm_topic", 10, std::bind(&DashboardController::getPWM, this, std::placeholders::_1),PWMOptions);

    SoftwareKSSub = 
    this->create_subscription<std_msgs::msg::Bool>("killstatus_topic", 10, 
    std::bind(&DashboardController::getSoftwareKS, this, std::placeholders::_1), SoftwareKSOptions);


    //Publishers
    X_Axis_Publisher = this->create_publisher<std_msgs::msg::Float32MultiArray>("X_axis_CLTool_Topic", 10); 
    Y_Axis_Publisher = this->create_publisher<std_msgs::msg::Float32MultiArray>("Y_axis_CLTool_Topic", 10); 
    Z_Axis_Publisher = this->create_publisher<std_msgs::msg::Float32MultiArray>("Z_axis_CLTool_Topic", 10); 
    ROLL_Axis_Publisher = this->create_publisher<std_msgs::msg::Float32MultiArray>("roll_axis_CLTool_Topic", 10); 
    PITCH_Axis_Publisher = this->create_publisher<std_msgs::msg::Float32MultiArray>("pitch_axis_CLTool_Topic", 10); 
    YAW_Axis_Publisher =  this->create_publisher<std_msgs::msg::Float32MultiArray>("yaw_axis_CLTool_Topic", 10); 
    // Vision
}
void DashboardController::Controller()
{
    while (!isControllerShutdown.load())
    {
        // Robot not Running, See if Running
        if (ComponentStruct->LocationData.CurrentTask.load(std::memory_order_acquire) != nullptr)
        {
            isRobotRunning = true;
            std::lock_guard<std::mutex>(ComponentStruct->SystemStatusmutex);
            // std::fill(ComponentStruct->SystemStatusData.begin(), ComponentStruct->SystemStatusData.end(), "success");
            for (auto i : ComponentStruct->SystemStatusData)
            {
                i.status = Success;
            }
        }
        if (isRobotRunning)
        {
            // Generate SystemStatusData vector
            std::lock_guard<std::mutex>(ComponentStruct->SystemStatusmutex);
            for (auto i : ComponentStruct->SystemStatusData)
            {
                if (i.status == Danger)
                {
                    ComponentStruct->SystemStatusData[0].status = Danger;
                    ComponentStruct->SystemStatusData[0].message = i.message;
                    break;
                }
            }
        }
        if(!isRobotRunning){
        std::lock_guard<std::mutex>(ComponentStruct->SystemStatusmutex);
        for (auto i : ComponentStruct->SystemStatusData)
        {
            i.status = Standby;
        }
    }
        //DO REGARDLESS
        auto msg = std_msgs::msg::Float32MultiArray();
        msg.data.resize(8);
        Axis axis = ComponentStruct->Axes[0];
        for(int i = 0; i < 8; i++){
                std::lock_guard<std::mutex> lk(axis.PID_Components[i]->mutex);
                msg.data[i] = axis.PID_Components[i]->value;
        }
        X_Axis_Publisher->publish(msg);
    
        axis = ComponentStruct->Axes[1];
        for(int i = 0; i < 8; i++){
                std::lock_guard<std::mutex> lk(axis.PID_Components[i]->mutex);
                msg.data[i] = axis.PID_Components[i]->value;
        }
        Y_Axis_Publisher->publish(msg);
    
        axis = ComponentStruct->Axes[2];
        for(int i = 0; i < 8; i++){
                std::lock_guard<std::mutex> lk(axis.PID_Components[i]->mutex);
                msg.data[i] = axis.PID_Components[i]->value;
        }
        Z_Axis_Publisher->publish(msg);
    
        axis = ComponentStruct->Axes[3];
        for(int i = 0; i < 8; i++){
                std::lock_guard<std::mutex> lk(axis.PID_Components[i]->mutex);
                msg.data[i] = axis.PID_Components[i]->value;
        }
        ROLL_Axis_Publisher->publish(msg);
    
        axis = ComponentStruct->Axes[4];
        for(int i = 0; i < 8; i++){
                std::lock_guard<std::mutex> lk(axis.PID_Components[i]->mutex);
                msg.data[i] = axis.PID_Components[i]->value;
        }
        PITCH_Axis_Publisher->publish(msg);
    
        axis = ComponentStruct->Axes[5];
        for(int i = 0; i < 8; i++){
                std::lock_guard<std::mutex> lk(axis.PID_Components[i]->mutex);
                msg.data[i] = axis.PID_Components[i]->value;
        }
        YAW_Axis_Publisher->publish(msg);
   
    }
}

    //

void DashboardController::getSOC(const std_msgs::msg::Float64::SharedPtr msg)
{
    ComponentStruct->BatteryData.SOC.store(msg->data, std::memory_order_release);
    // std::cout << SOC << std::endl;
    // Need to setup GUI
}
void DashboardController::getSOCINT(const std_msgs::msg::Bool::SharedPtr msg)
{
    std::lock_guard<std::mutex>(ComponentStruct->SystemStatusmutex);
    ComponentStruct->SystemStatusData[1].status = Danger;
    ComponentStruct->SystemStatusData[1].message = "SOC IS LOW -> SOC INT IS TRIGGERED";
    ComponentStruct->BatteryData.SOCint.store(msg->data, std::memory_order_release);
}

void DashboardController::getWaypoint(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    std::array<float, 6> arr;
    std::copy_n(msg->data.begin(), 6, arr.begin());
    ComponentStruct->LocationData.CurrentWaypoint.store(std::make_shared<Position>(arr), std::memory_order_release);
}
void DashboardController::getPosition(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    std::array<float, 6> arr;
    std::copy_n(msg->data.begin(), 6, arr.begin());
    ComponentStruct->LocationData.CurrentPosition.store(std::make_shared<Position>(arr), std::memory_order_release);
}
void DashboardController::getVolt(const std_msgs::msg::Float64::SharedPtr msg){
	ComponentStruct->BatteryData.battery_voltage.store(msg->data, std::memory_order_release);
}
void DashboardController::getCurrentTask(const std_msgs::msg::String::SharedPtr msg)
{
    ComponentStruct->LocationData.CurrentTask.store(std::make_shared<std::string>(msg->data), std::memory_order_release);
}
void DashboardController::getPWM(const std_msgs::msg::Int32MultiArray::SharedPtr msg){
	std::array<int, 8> arr;
	std::copy_n(msg->data.begin(),  8, arr.begin());
	ComponentStruct->ThrustData.CurrentPWM.store(std::make_shared<std::array<int,8>>(arr), std::memory_order_release);
}
void DashboardController::getSoftwareKS(const std_msgs::msg::Bool::SharedPtr msg){
    isSoftwareKS = msg->data;
    std::lock_guard<std::mutex>(ComponentStruct->SystemStatusmutex);
    if(isSoftwareKS) {
    ComponentStruct->SystemStatusData[3].status = Danger;
    ComponentStruct->SystemStatusData[3].message = "Software Kill Switch Triggered.";
    ComponentStruct->SystemStatusData[0].message = "Robot is currently software-killed.";
    }else{
    ComponentStruct->SystemStatusData[3].status = Success;
    ComponentStruct->SystemStatusData[3].message = "Software Kill Switch NOT Triggered";
    }
}
// getposition

void DashboardController::Shutdown()
{
    isControllerShutdown = true;
}
