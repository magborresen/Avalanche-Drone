// Import relevant libraries from the SDK
#include "StepResponse.hpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK:Telemetry;


Telemetry::Quaternion quarternion;

std::fstream myFile;
std::string filename = "logFile.txt";
// Create a function to get the yaw that takes the vehicle as a parameter and a timeout parameter
void getYaw (Vehicle* vehicle, int timeoutParamInMs)
{
	float yaw;
	int elapsedTimeInMs = 0;
	while(elapsedTimeInMs < timeoutParamInMs)
	{
		quarternion = vehicle->broadcast->getQuarternion();
		yaw = 2*argsin(quarternion.q3);
		appendToFile(yaw);
		usleep(5000);
		elapsedTimeInMs += 5;
	}
}


// Setup telemetry variables (see line 57 in the telemetry sample)




// Setup broadcast frequency




// Get the quartenion and convert to yaw





// Create a function that can open and save data to a file that takes the yaw as the parameter
void openFile()
{
    myfile.open(filename,  std::ofstream::out | std::ofstream::trunc);
    myfile.close();
}

void appendToFile(float input)
{
	myFile.open(filename, std::ios_base::app);
	myFile << input;
	myFile << "; ";
	myFile.close();
}
	


// Create a function that gives the yaw a step response

bool doStep()
{
	openFile();
	WayPointInitSettings fdata;
	
}