#include <atomic>
struct StructofComponents{
    /*
      std::shared_ptr<Name of Battery Struct>;
      and more sharedptr<etc...>
    */
};

struct BatteryStruct{
    std::atomic<float> SOC;
    std::atmoic<bool> SOCint{false};
};
struct LocationStruct{
    std::atomic<std::shared_ptr<std::string>> CurrentTask;
    std::atomic<std::shared_ptr<std::array<float,6>>> CurrentWaypoint;
}
struct SystemStatus {
    std::string name;     // Name of the system (e.g., "Manipulator", "Vision")
    std::string status;   // "error", "warning", "success"
    std::string message;  // e.g., "system is connected"
};
struct ManipulationStruct{
    std::atomic<int> ManipulationCode;
}