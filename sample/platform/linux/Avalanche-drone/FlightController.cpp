#include <cmath>

// DJI OSDK includes
#include "dji_status.hpp"
#include <dji_vehicle.hpp>

#include "FlightController.hpp"

bool monitoredTakeoff(Vehicle* vehicle, int timeout){
    char func[50];
    // Start takeoff
    ACK::ErrorCode takeoffStatus = vehicle->control->takeoff(timeout);
    if (ACK::getError(takeoffStatus) != ACK::SUCCESS)
    {
        ACK::getErrorCodeMessage(takeoffStatus, func);
        return false;
    }

    // First check: Motors started
    int motorsNotStarted = 0;
    int timeoutCycles    = 20;

  
    while ((vehicle->broadcast->getStatus().flight < DJI::OSDK::VehicleStatus::M100FlightStatus::TAKEOFF) && motorsNotStarted < timeoutCycles)    {
      motorsNotStarted++;
      usleep(100000);
    }

    if (motorsNotStarted < timeoutCycles)    {
      std::cout << "Successful TakeOff!" << std::endl;
    }

    // Second check: In air
    int stillOnGround = 0;
    timeoutCycles     = 110;

    while ((vehicle->broadcast->getStatus().flight != DJI::OSDK::VehicleStatus::M100FlightStatus::IN_AIR_STANDBY) && stillOnGround < timeoutCycles){
      stillOnGround++;
      usleep(100000);
    }

    if (stillOnGround < timeoutCycles)    {
      std::cout << "Aircraft in air!" << std::endl;
    }
  

    // Final check: Finished takeoff
    float32_t                 delta;
    Telemetry::GlobalPosition currentHeight;
    Telemetry::GlobalPosition deltaHeight =
    vehicle->broadcast->getGlobalPosition();
    do {
      sleep(4);
      currentHeight = vehicle->broadcast->getGlobalPosition();
      delta         = fabs(currentHeight.altitude - deltaHeight.altitude);
      deltaHeight.altitude = currentHeight.altitude;
    } while (delta >= 0.009);
    std::cout << "Aircraft hovering at " << currentHeight.altitude << "m!\n";
    // Cleanup
  return true;
}