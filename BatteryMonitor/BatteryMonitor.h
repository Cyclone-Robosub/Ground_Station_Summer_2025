#include "BatteryMonitor.cpp"
#include <chrono>

#define CURRENTRATING 10.5

class BatteryMonitor {
public:
  BatteryMonitor() {
		timeInital = startupTime;
   // SetupPicoConnection();
  };

  // Setup publisher to a SOC Topic.
private:
  std::chrono::steady_clock::time_point startupTime = std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point timeInital;
  double ReadSOC();
};