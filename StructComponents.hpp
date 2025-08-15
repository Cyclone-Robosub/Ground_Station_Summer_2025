#pragma once
#include <atomic>
#include <memory>
#include <mutex>
#include <vector>
#include "include/crs_common/position/position.hpp"



struct BatteryStruct
{
    std::atomic<float> battery_voltage;
    std::atomic<float> battery_threshold{13.8f};
    std::atomic<float> SOC;
    std::atomic<bool> SOCint{false};
    // std::atomic<StatusCondition> BatteryStatus;
};
struct LocationStruct
{
    std::atomic<std::shared_ptr<std::string>> CurrentTask;
    std::atomic<std::shared_ptr<Position>> CurrentWaypoint;
    std::atomic<std::shared_ptr<Position>> CurrentPosition;
};
struct ThrustStruct
{
	std::atomic<std::shared_ptr<std::array<int,8>>> CurrentPWM;
};
/*
struct SystemStatuses {
    //the index determines which ComponentStruct we are talking about.
    std::mutex SystemStatusmutex;
    constexpr const int numOfComponents = 3;
    StatusCondition MainStatus;
    std::vector<std::string> names;     // Name of the system (e.g., "Manipulator", "Vision")
    std::vector<StatusCondition> StatusVector{3, StatusCondition::STANDBY};  // "DANGER", "warning", "success", "standby"
    std::vector<std::string> messages;  // e.g., "system is connected"
};*/

enum Status{
    Danger, Warning, Success, Standby
	
};
std::string StatusToString(Status givenStatus);
struct SystemStatus
{
    std::string name;    // Name of the system (e.g., "Manipulator", "Vision")
    Status status;  // "Danger", "warning", "success", "standby"
    std::string message; // e.g., "system is connected"
};
struct ManipulationStruct
{
    std::atomic<int> ManipulationCode;
};
struct CL_Tool_Struct
{
    std::mutex CLToolDataMutex;
    std::shared_ptr<float[]> x_axis_array;
    std::shared_ptr<float[]> y_axis_array;
    std::shared_ptr<float[]> z_axis_array;
    std::shared_ptr<float[]> roll_array;
    std::shared_ptr<float[]> pitch_array;
    std::shared_ptr<float[]> yaw_array;
};
struct StructofComponents
{
    BatteryStruct BatteryData;
    // and more sharedptr<etc...>
    std::mutex SystemStatusmutex;
    std::vector<SystemStatus> SystemStatusData = {
        {"Master Status", Standby, "System is on standby"},
        {"Battery System", Standby, "System is on standby"},
        {"Location System", Standby, "System is on standby"},
        {"Software Kill Switch Status", Standby, "System is on standby"}
    };
    LocationStruct LocationData;
    ThrustStruct ThrustData;
    Cl_Tool_Struct CLToolData;
};
