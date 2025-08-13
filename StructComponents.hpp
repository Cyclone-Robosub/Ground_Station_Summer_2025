#pragma once
#include <atomic>
#include <memory>
#include <mutex>
#include <vector>
#include "../crs_common/position/position.hpp"
struct BatteryStruct
{
    std::atomic<float> battery_voltage;
    std::atomic<float> battery_threshold{13.3f};
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
struct SystemStatus
{
    std::string name;    // Name of the system (e.g., "Manipulator", "Vision")
    std::string status;  // "Danger", "warning", "success", "standby"
    std::string message; // e.g., "system is connected"
};
struct ManipulationStruct
{
    std::atomic<int> ManipulationCode;
};

struct StructofComponents
{
    BatteryStruct BatteryData;
    // and more sharedptr<etc...>
    std::mutex SystemStatusmutex;
    std::vector<SystemStatus> SystemStatusData = {
        {"Master Status", "standby", "System is on standby"},
        {"Battery System", "standby", "System is on standby"},
        {"Location System", "standby", "System is on standby"}};
    LocationStruct LocationData;
};
