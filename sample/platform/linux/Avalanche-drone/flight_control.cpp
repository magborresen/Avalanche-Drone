#include "flight_control.hpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;



bool monitoredTakeoff(Vehicle* vehicle, int timeout)
{
	char func[50];
	int  pkgIndex;

	// Start takeoff - starts by creating an error code to hold aircraft takeoff status
	ACK::ErrorCode takeoffStatus = vehicle->control->takeoff(timeout);

	// If takeoff is not successful
	if (ACK::getError(takeoffStatus) != ACK::SUCCESS)
	{
		ACK::getErrorCodeMessage(takeoffStatus, func);
		return false;
	}

	// First check: Motors started
	int motorsNotStarted = 0;
	int timeoutCycles    = 20;

	// While loop checks if the vehicle is in flight and holds it up to the timeout
	while ((vehicle->broadcast->getStatus().flight < DJI::OSDK::VehicleStatus::M100FlightStatus::TAKEOFF) && motorsNotStarted < timeoutCycles)
	{
		motorsNotStarted++;
		usleep(100000);
	}

	if (motorsNotStarted < timeoutCycles)
	{
		std::cout << "Successful TakeOff!" << std::endl;
	}

	// Second check: Is the vehicle in the air?
	int stillOnGround = 0;
	timeoutCycles     = 110;


	while ((vehicle->broadcast->getStatus().flight != DJI::OSDK::VehicleStatus::M100FlightStatus::IN_AIR_STANDBY) && stillOnGround < timeoutCycles)
	{
		stillOnGround++;
		usleep(100000);
	}

	if (stillOnGround < timeoutCycles)
	{
		std::cout << "Aircraft in air!" << std::endl;
	}

	// Final check: Is takeoff finished?
	float32_t delta;
	Telemetry::GlobalPosition currentHeight;
	Telemetry::GlobalPosition deltaHeight = vehicle->broadcast->getGlobalPosition();

	do
	{
		sleep(4);
		currentHeight = vehicle->broadcast->getGlobalPosition();
		delta = fabs(currentHeight.altitude - deltaHeight.altitude);
		deltaHeight.altitude = currentHeight.altitude;
	} while (delta >= 0.009);

	std::cout << "Aircraft hovering at " << currentHeight.altitude << "m!\n";

	return true;
}

bool moveByPositionOffset(Vehicle *vehicle, float xOffsetDesired, float yawDesired, float posThresholdInM, float yawThresholdInDeg)
{
	// Set timeout: this timeout is the time you allow the drone to take to finish the mission
	int responseTimeout              = 1;
	int timeoutInMilSec              = 40000;
	int controlFreqInHz              = 50; // Hz
	int cycleTimeInMs                = 1000 / controlFreqInHz;
	int outOfControlBoundsTimeLimit  = 10 * cycleTimeInMs; // 10 cycles
	int withinControlBoundsTimeReqmt = 50 * cycleTimeInMs; // 50 cycles
	int pkgIndex;
	
	char func[50];
	
	float yOffsetDesired, zOffsetDesired = 0;
	
	vehicle->control->velocityAndYawRateCtrl(10, 10, 1, 100);

	// Get data

	// Global position retrieved via broadcast
	Telemetry::GlobalPosition currentBroadcastGP;
	Telemetry::GlobalPosition originBroadcastGP;

	// Convert position offset from first position to local coordinates
	Telemetry::Vector3f localOffset;
	
	currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
	originBroadcastGP  = currentBroadcastGP;
	localOffsetFromGpsOffset(vehicle, localOffset, static_cast<void*>(&currentBroadcastGP), static_cast<void*>(&originBroadcastGP));


	// Get initial offset. Updated in the loop later
	double xOffsetRemaining = xOffsetDesired - localOffset.x;
	double yOffsetRemaining = yOffsetDesired - localOffset.y;
	double zOffsetRemaining = zOffsetDesired - localOffset.z;

	// Conversions
	double yawDesiredRad     = DEG2RAD * yawDesired;
	double yawThresholdInRad = DEG2RAD * yawThresholdInDeg;

	//! Get Euler angle

	// Quaternion retrieved via broadcast
	Telemetry::Quaternion broadcastQ;

	double yawInRad;
	
	// Get and store the vehicles current yaw angle. Converted to radians.
	broadcastQ = vehicle->broadcast->getQuaternion();
	yawInRad   = toEulerAngle((static_cast<void*>(&broadcastQ))).z / DEG2RAD;

	int   elapsedTimeInMs     = 0;
	int   withinBoundsCounter = 0;
	int   outOfBounds         = 0;
	int   brakeCounter        = 0;
	int   speedFactor         = 2;
	float xCmd, yCmd, zCmd;
	/*! Calculate the inputs to send the position controller. We implement basic
	*  receding setpoint position control and the setpoint is always 1 m away
	*  from the current position - until we get within a threshold of the goal.
	*  From that point on, we send the remaining distance as the setpoint.
	*/
	if (xOffsetDesired > 0)
	xCmd = (xOffsetDesired < speedFactor) ? xOffsetDesired : speedFactor;
	else if (xOffsetDesired < 0)
	xCmd = (xOffsetDesired > -1 * speedFactor) ? xOffsetDesired : -1 * speedFactor;
	else
	xCmd = 0;

	if (yOffsetDesired > 0)
	yCmd = (yOffsetDesired < speedFactor) ? yOffsetDesired : speedFactor;
	else if (yOffsetDesired < 0)
	yCmd = (yOffsetDesired > -1 * speedFactor) ? yOffsetDesired : -1 * speedFactor;
	else
	yCmd = 0;
	
    zCmd = currentBroadcastGP.height + zOffsetDesired;
	
	// Initialize control data struct. 
	Control::CtrlData controlData = Control::CtrlData((Control::HorizontalLogic::HORIZONTAL_VELOCITY | Control::VerticalLogic::VERTICAL_VELOCITY | Control::YawLogic::YAW_ANGLE 
						| Control::HorizontalCoordinate::HORIZONTAL_BODY | Control::StableMode::STABLE_ENABLE), xCmd, yCmd, zCmd, yawDesiredRad / DEG2RAD);

	//! Main closed-loop receding set-point position control
	while (elapsedTimeInMs < timeoutInMilSec)
	{
		vehicle->control->flightCtrl(controlData);

		usleep(cycleTimeInMs * 1000);
		elapsedTimeInMs += cycleTimeInMs;

		//! Get current position in required coordinates and units

		broadcastQ         = vehicle->broadcast->getQuaternion();
		yawInRad           = toEulerAngle((static_cast<void*>(&broadcastQ))).z;
		currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
		localOffsetFromGpsOffset(vehicle, localOffset, static_cast<void*>(&currentBroadcastGP), static_cast<void*>(&originBroadcastGP));

		//! See how much farther we have to go
		xOffsetRemaining = xOffsetDesired - localOffset.x;
		yOffsetRemaining = yOffsetDesired - localOffset.y;
		zOffsetRemaining = zOffsetDesired - localOffset.z;

		//! See if we need to modify the setpoint
		if (std::abs(xOffsetRemaining) < speedFactor)
		{
		  controlData.x = xOffsetRemaining;
		}
		if (std::abs(yOffsetRemaining) < speedFactor)
		{
		  controlData.y = yOffsetRemaining;
		}

		if (std::abs(xOffsetRemaining) < posThresholdInM &&
			std::abs(yOffsetRemaining) < posThresholdInM &&
			std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)
		{
		  //! 1. We are within bounds; start incrementing our in-bound counter
		  withinBoundsCounter += cycleTimeInMs;
		}
		else if (std::abs(xOffsetRemaining) < posThresholdInM &&
				 std::abs(yOffsetRemaining) < posThresholdInM &&
				 std::abs(zOffsetRemaining) < posThresholdInM &&
				 std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)
		{
		  //! 1. We are within bounds; start incrementing our in-bound counter
		  withinBoundsCounter += cycleTimeInMs;
		}
		else
		{
		  if (withinBoundsCounter != 0)
		  {
			//! 2. Start incrementing an out-of-bounds counter
			outOfBounds += cycleTimeInMs;
		  }
		}
		//! 3. Reset withinBoundsCounter if necessary
		if (outOfBounds > outOfControlBoundsTimeLimit)
		{
		  withinBoundsCounter = 0;
		  outOfBounds         = 0;
		}
		//! 4. If within bounds, set flag and break
		if (withinBoundsCounter >= withinControlBoundsTimeReqmt)
		{
		  break;
		}
	}

		if (elapsedTimeInMs >= timeoutInMilSec)
		{
		std::cout << "Task timeout!\n";
	}

	return ACK::SUCCESS;
}

/*! Monitored Takeoff (Blocking API call). Return status as well as ack.
    This version of takeoff makes sure your aircraft actually took off
    and only returns when takeoff is complete.
    Use unless you want to do other stuff during takeoff - this will block
    the main thread.
!*/
bool monitoredLanding(Vehicle* vehicle, int timeout)
{
	
	char func[50];
	int  pkgIndex;

	// Start landing
	ACK::ErrorCode landingStatus = vehicle->control->land(timeout);
	if (ACK::getError(landingStatus) != ACK::SUCCESS)
	{
	ACK::getErrorCodeMessage(landingStatus, func);
	return false;
	}

	// First check: Landing started
	int landingNotStarted = 0;
	int timeoutCycles     = 20;

	// Check if landing starts. If it hits timeout, move on
    while (vehicle->broadcast->getStatus().flight != DJI::OSDK::VehicleStatus::M100FlightStatus::LANDING && landingNotStarted < timeoutCycles)
    {
		landingNotStarted++;
		usleep(100000);
    }

	// Check if landing is started, is it smaller than timeoutCycles?
	if (landingNotStarted == timeoutCycles)
	{
		std::cout << "Landing failed. Aircraft is still in the air." << std::endl;
		return false;
	}
	// Landing has started
	else
	{
		std::cout << "Landing...\n";
	}

	// Wait until landing is finished
    while (vehicle->broadcast->getStatus().flight == DJI::OSDK::VehicleStatus::M100FlightStatus::FINISHING_LANDING)
    {
		sleep(1);
    }
	
	// Get the global position
    Telemetry::GlobalPosition gp;
    do
    {
	  sleep(2);
	  gp = vehicle->broadcast->getGlobalPosition();
    } while (gp.altitude != 0);
	
	// Check the mode of the vehicle
    if (gp.altitude != 0)
    {
		std::cout << "Landing finished, but the aircraft is in an unexpected mode. Please connect DJI GO.\n";
		return false;
    }
	else
	{
		std::cout << "Successful landing!\n";
	}

  return true;
}

// Helper Functions

/*! Very simple calculation of local NED offset between two pairs of GPS
/coordinates.
    Accurate when distances are small.
!*/
void localOffsetFromGpsOffset(Vehicle* vehicle, Telemetry::Vector3f& deltaNed, void* target, void* origin)
{
	Telemetry::GPSFused*       subscriptionTarget;
	Telemetry::GPSFused*       subscriptionOrigin;
	Telemetry::GlobalPosition* broadcastTarget;
	Telemetry::GlobalPosition* broadcastOrigin;
	double deltaLon;
	double deltaLat;

	broadcastTarget = (Telemetry::GlobalPosition*)target;
	broadcastOrigin = (Telemetry::GlobalPosition*)origin;
	deltaLon        = broadcastTarget->longitude - broadcastOrigin->longitude;
	deltaLat        = broadcastTarget->latitude - broadcastOrigin->latitude;
	deltaNed.x      = deltaLat * C_EARTH;
	deltaNed.y      = deltaLon * C_EARTH * cos(broadcastTarget->latitude);
	deltaNed.z      = broadcastTarget->altitude - broadcastOrigin->altitude;
}

Telemetry::Vector3f toEulerAngle(void* quaternionData)
{
  Telemetry::Vector3f    ans;
  Telemetry::Quaternion* quaternion = (Telemetry::Quaternion*)quaternionData;

  double q2sqr = quaternion->q2 * quaternion->q2;
  double t0    = -2.0 * (q2sqr + quaternion->q3 * quaternion->q3) + 1.0;
  double t1 =
    +2.0 * (quaternion->q1 * quaternion->q2 + quaternion->q0 * quaternion->q3);
  double t2 =
    -2.0 * (quaternion->q1 * quaternion->q3 - quaternion->q0 * quaternion->q2);
  double t3 =
    +2.0 * (quaternion->q2 * quaternion->q3 + quaternion->q0 * quaternion->q1);
  double t4 = -2.0 * (quaternion->q1 * quaternion->q1 + q2sqr) + 1.0;

  t2 = (t2 > 1.0) ? 1.0 : t2;
  t2 = (t2 < -1.0) ? -1.0 : t2;

  ans.x = asin(t2);
  ans.y = atan2(t3, t4);
  ans.z = atan2(t1, t0);

  return ans;
}
