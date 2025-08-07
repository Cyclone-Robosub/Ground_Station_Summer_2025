
#include <atomic>
#include <memory>
#include <mutex>
#include <vector>
struct BatteryStruct{
    std::atomic<float> battery_voltage;  
    std::atomic<float> battery_threshold{13.3f};
    std::atomic<float> SOC;
    std::atomic<bool> SOCint{false};
};
struct LocationStruct{
    std::atomic<std::shared_ptr<std::string>> CurrentTask;
    std::atomic<std::shared_ptr<std::array<float,6>>> CurrentWaypoint;
    std::atomic<std::shared_ptr<std::array<float,6>>> CurrentPosition;
}
struct SystemStatus {
    //the index determines which ComponentStruct we are talking about.
  //  std::shared_ptr<std::mutex> SystemStatusmutex;
    std::string MainStatus;
    //std::vector<std::string> names;     // Name of the system (e.g., "Manipulator", "Vision")
    std::vector<std::atomic<int>> statuses(3,std::atomic<int>(3));   // "error", "warning", "success", "standby"
    //std::vector<std::string> messages;  // e.g., "system is connected"
};
struct ManipulationStruct{
    std::atomic<int> ManipulationCode;
}


struct StructofComponents{
      BatteryStruct BatteryData;
      //and more sharedptr<etc...>
      SystemStatus SystemStatusData;
      LocationStruct LocationData;
};
