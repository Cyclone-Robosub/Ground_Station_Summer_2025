#pragma once
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <vector>
#include "include/crs_common/position/position.hpp"

constexpr int numSubsystems = 4;

//! decide on mutable keyword

struct BatteryStruct
{
    float battery_voltage{0.0f};

    float battery_threshold{13.8f};

    float SOC{0.0f};

    bool SOCint{false};

    std::mutex mtx_battery;
};

struct LocationStruct
{
    std::shared_mutex mtx_CurrentTask;
    std::shared_ptr<std::string> CurrentTask;

    std::shared_mutex mtx_CurrentWaypoint;
    std::shared_ptr<Position> CurrentWaypoint;

    std::shared_mutex mtx_CurrentPosition;
    std::shared_ptr<Position> CurrentPosition;
};

struct ThrustStruct
{
    std::mutex mtx_CurrentPWM;
    std::shared_ptr<std::array<int, 8>> CurrentPWM;
};

enum Status
{
    Danger,
    Warning,
    Success,
    Standby
};

std::string StatusToString(Status givenStatus);

struct SystemStatus
{
    std::string name;    // Name of the system (e.g., "Manipulator", "Vision")
    Status status;       // "Danger", "Warning", "Success", "Standby"
    std::string message; // e.g., "system is connected"

    std::mutex mtx_system_status;
};

struct ManipulationStruct
{
    std::mutex mtx_ManipulationCode;
    int ManipulationCode{0};
};

struct StructofComponents
{
    BatteryStruct BatteryData;

    std::mutex SystemStatusmutex;
    std::array<SystemStatus, numSubsystems> SystemStatusArray = {
        {"Master Status", Standby, "System is on standby"},
        {"Battery System", Standby, "System is on standby"},
        {"Location System", Standby, "System is on standby"},
        {"Software Kill Switch Status", Standby, "System is on standby"}
    };

    LocationStruct LocationData;
    ThrustStruct ThrustData;
};
