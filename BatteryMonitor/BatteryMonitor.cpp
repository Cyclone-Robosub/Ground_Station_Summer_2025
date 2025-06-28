#include "BatteryMonitor.h"
double BatteryMonitor::ReadSOC() {
  double currentCurrent;
  std::chrono::steady_clock::time_point timeFinal;
  const double currentRating = CURRENTRATING;
  double previousSOC;
  double resultingSOC;
  // Get Time Final
  //  wait and set value for current value and Time Final.
  double currentCurrent = // get READING;
      timeFinal = std::chrono::steady_clock::now();
  auto deltaTime = std::chrono::duration<double>(timeFinal - timeInital).count();
  resultingSOC =
      previousSOC * (currentCurrent / currentRating) * (deltaTime);

  // Setup for next Reading of SOC
  previousSOC = resultingSOC;
  timeInital = timeFinal;

  return resultingSOC;
}