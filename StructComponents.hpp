#pragma once
#include <atomic>
#include <memory>
#include <mutex>
#include <vector>
enum class StatusCondition {
    DANGER   = 0,
    WARNING = 1,
    SUCCESS = 2,
    STANDBY = 3
};

struct BatteryStruct{
    std::atomic<float> battery_voltage;  
    std::atomic<float> battery_threshold{13.3f};
    std::atomic<float> SOC;
    std::atomic<bool> SOCint{false};
    std::atomic<StatusCondition> BatteryStatus;
};
struct LocationStruct{
    std::atomic<std::shared_ptr<std::string>> CurrentTask;
    std::atomic<std::shared_ptr<std::array<float,6>>> CurrentWaypoint;
    std::atomic<std::shared_ptr<std::array<float,6>>> CurrentPosition;
};
struct SystemStatuses {
    //the index determines which ComponentStruct we are talking about.
    std::mutex SystemStatusmutex;
    StatusCondition MainStatus;
    //std::vector<std::string> names;     // Name of the system (e.g., "Manipulator", "Vision")
    std::vector<StatusCondition> StatusVector{3, StatusCondition::STANDBY};  // "DANGER", "warning", "success", "standby"
    //std::vector<std::string> messages;  // e.g., "system is connected"
};
struct ManipulationStruct{
    std::atomic<int> ManipulationCode;
};


struct StructofComponents{
      BatteryStruct BatteryData;
      //and more sharedptr<etc...>
      SystemStatuses SystemStatusData;
      LocationStruct LocationData;
};
