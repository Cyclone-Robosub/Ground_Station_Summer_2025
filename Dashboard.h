#include "BatteryMonitor/BatteryMonitor.h"
#include <chrono>
#include <iostream>
#include <thread>

class Dashboard {
public:
  Dashboard() {
    std::cout << "Startup time : " << startuptime << std::endl;
    (BatteryMonitor(startuptime));
  }
  std::chrono::system_clock::time_point getStartupTime() { return startuptime; }
  
private:
  const std::chrono::system_clock::time_point startuptime =
      std::chrono::system_clock::now();
      //Setup subscriber to SOC Topic.
    UpdateDashboardValues();
    //Decide on Event Driven vs Real-Time System.
};