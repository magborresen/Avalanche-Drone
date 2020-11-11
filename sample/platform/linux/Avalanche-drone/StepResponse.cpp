// Import relevant libraries from the SDK
#include "StepResponse.hpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;




std::fstream myFile;
std::string logFile = "logFile.txt";
// Create a function to get the yaw that takes the vehicle as a parameter and a timeout parameter

void clearFile()
{
    myFile.open(logFile,  std::ofstream::out | std::ofstream::trunc);
    myFile.close();
}

void appendToFile(float yaw, uint32_t timeStamp)
{
	myFile.open(logFile, std::ios_base::app);
	myFile << yaw << "," << timeStamp;
	myFile << "\n";
	myFile.close();
}

void getYaw(Vehicle* vehicle, int timeoutParamInMs)
{
	Telemetry::Quaternion quaternion;
	Telemetry::TimeStamp timeStamp;
	const int TIMEOUT = 20;
	ACK::ErrorCode ack = vehicle->broadcast->setBroadcastFreqDefaults(TIMEOUT);
	
	float yaw;
	int elapsedTimeInMs = 0;
	while(elapsedTimeInMs < timeoutParamInMs)
	{
		quaternion = vehicle->broadcast->getQuaternion();
		timeStamp = vehicle->broadcast->getTimeStamp();
		yaw = 2*asin(quaternion.q3);
		appendToFile(yaw, timeStamp.time_ms);
		usleep(5000);
		elapsedTimeInMs += 5;
	}
}


// Create a function that gives the yaw a step response

void doStep(Vehicle* vehicle)
{
	float xCmd = 0;
	float yCmd = 0;
	float zCmd = 0;
	float yawCmd = 150;
	
	float xPos = 0;
	float yPos = 0;
	float zPos = 2;
	float yawPos = 0;
	
	
	int measuringTimeMs = 5000;
	
	clearFile();
	std::cout << "Taking off...";
	vehicle->control->takeoff();
	std::cout << "TakeOff succeeded...";
	std::cout << "Setting Yaw rate...";
	vehicle->control->velocityAndYawRateCtrl(xCmd, yCmd, zCmd, yawCmd);
	std::cout << "Yaw Rate set...";
	std::cout << "Rotating";
	vehicle->control->positionAndYawCtrl(xPos, yPos, zPos, yawPos);
	msleep(5000);
	yawpos = 180;
	vehicle->control->positionAndYawCtrl(xPos, yPos, zPos, yawPos);
	getYaw(vehicle, measuringTimeMs);
	
	
}