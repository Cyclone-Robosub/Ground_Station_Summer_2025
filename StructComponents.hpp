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


class PID_Component{
    public:
    PID_Component(const std::string& n, float v) : name(n), value(v) {}
    PID_Component() = default;
    // Explicitly delete copy constructor and assignment operator to prevent copies.
    PID_Component(const PID_Component&) = delete;
    PID_Component& operator=(const PID_Component&) = delete;
     std::string name;
    std::mutex mutex;
    float value;
};
struct Axis{
    std::array<std::shared_ptr<PID_Component>, 8> PID_Components;
    std::string name;
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
   std::array<Axis, 6> Axes =
       {
            Axis{{ std::make_shared<PID_Component>("p", 1.0f),
                   std::make_shared<PID_Component>("i", 0.05f),
                   std::make_shared<PID_Component>("d", 0.5f),
                   std::make_shared<PID_Component>("value4", 1000.0f),
                   std::make_shared<PID_Component>("value5", 0.5f),
                   std::make_shared<PID_Component>("value6", 0.05f),
                   std::make_shared<PID_Component>("value7", 0.1f),
                   std::make_shared<PID_Component>("value8", 5.0f) }, "x-axis"},
            Axis{{ std::make_shared<PID_Component>("p", 1.0f),
                   std::make_shared<PID_Component>("i", 0.05f),
                   std::make_shared<PID_Component>("d", 0.5f),
                   std::make_shared<PID_Component>("value4", 1000.0f),
                   std::make_shared<PID_Component>("value5", 0.5f),
                   std::make_shared<PID_Component>("value6", 0.05f),
                   std::make_shared<PID_Component>("value7", 0.1f),
                   std::make_shared<PID_Component>("value8", 5.0f) }, "y-axis"},
            Axis{{ std::make_shared<PID_Component>("p", 1.0f),
                   std::make_shared<PID_Component>("i", 0.05f),
                   std::make_shared<PID_Component>("d", 0.5f),
                   std::make_shared<PID_Component>("value4", 1000.0f),
                   std::make_shared<PID_Component>("value5", 0.5f),
                   std::make_shared<PID_Component>("value6", 0.05f),
                   std::make_shared<PID_Component>("value7", 0.1f),
                   std::make_shared<PID_Component>("value8", 5.0f) }, "z-axis"},
            Axis{{ std::make_shared<PID_Component>("p", 0.005f),
                   std::make_shared<PID_Component>("i", 0.0005f),
                   std::make_shared<PID_Component>("d", 0.005f),
                   std::make_shared<PID_Component>("value4", 100.0f),
                   std::make_shared<PID_Component>("value5", 1.0f),
                   std::make_shared<PID_Component>("value6", 5.0f),
                   std::make_shared<PID_Component>("value7", 10.0f),
                   std::make_shared<PID_Component>("value8", 5.0f) }, "roll"},
            Axis{{ std::make_shared<PID_Component>("p", 0.005f),
                   std::make_shared<PID_Component>("i", 0.0005f),
                   std::make_shared<PID_Component>("d", 0.005f),
                   std::make_shared<PID_Component>("value4", 100.0f),
                   std::make_shared<PID_Component>("value5", 1.0f),
                   std::make_shared<PID_Component>("value6", 5.0f),
                   std::make_shared<PID_Component>("value7", 10.0f),
                   std::make_shared<PID_Component>("value8", 5.0f) }, "pitch"},
            Axis{{ std::make_shared<PID_Component>("p", 0.5f),
                   std::make_shared<PID_Component>("i", 0.05f),
                   std::make_shared<PID_Component>("d", 0.05f),
                   std::make_shared<PID_Component>("value4", 100.0f),
                   std::make_shared<PID_Component>("value5", 0.25f),
                   std::make_shared<PID_Component>("value6", 5.0f),
                   std::make_shared<PID_Component>("value7", 10.0f),
                   std::make_shared<PID_Component>("value8", 5.0f) }, "yaw"}
        };

};
