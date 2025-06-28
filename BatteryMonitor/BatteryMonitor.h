#include "BatteryMonitor.cpp"
#include <chrono>

#define CURRENTRATING 10.5

class BatteryMonitor {
public:
  BatteryMonitor(const std::chrono::system_clock::time_point givenStartupTime)
      : startupTime(givenStartupTime) {
   //NEED TO SETUP PARSE THIS! timeInital = giveStartupTime;
    SetupPicoConnection();
  };

  // Setup publisher to a SOC Topic.
private:
  std::chrono::system_clock::time_point startupTime;
  double currentCurrent;
  timeFinal;
  timeInital;
  // deltaTime;
  const double currentRating = CURRENTRATING;
  double previousSOC;
  double resultingSOC;

  SetupPicoConnection() { Loop(); }
  double ReadSOC() {
    //Get Time Final from PICO?
    // wait and set value for current value and Time Final.
    double currentCurrent = // get READING;
    timeFinal = //get READING;
    resultingSOC = previousSOC * (currentCurrent / currentRating) *
                       (timeFinal - timeInital);

    // Setup for next Reading of SOC
    previousSOC = resultingSOC;
    timeInital = timeFinal;

    return resultingSOC;
  }
};