#include "BatteryMonitor/BatteryMonitor.h"
#include <chrono>
#include <iostream>
#include <thread>

class Dashboard
{
public:
  Dashboard(
      const std::chrono::steady_clock::time_point givenStartupTime)
      : startuptime(givenStartupTime)
  {
    std::cout << "Startup time : " << startuptime << std::endl;
    BatteryMonitor batteryMonitorInst = new BatteryMonitor(startuptime);
  }
  std::chrono::steady_clock::time_point getStartupTime() { return startuptime; }
  void UpdateDashboardValues();

private:
  const auto startuptime =
      std::chrono::steady_clock::now();
  // Setup subscriber to SOC Topic.
  BatteryMonitor batteryMonitorInst;
  // Decide on Event Driven vs Real-Time System.
};