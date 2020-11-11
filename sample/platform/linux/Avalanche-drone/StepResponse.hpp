/*
    This header file is to make and monitor a 180 deg stepresponse around the yaw axis of the drone.
    The header contains the functions needed to complete this.
*/

#ifndef STEP_RESPONSE_HPP
#define STEP_RESPONSE_HPP

// System includes
#include <cmath>
#include <vector>

#include <iostream>
#include <fstream>

// DJI OSDK includes
#include <dji_vehicle.hpp>
#include <dji_telemetry.hpp>
#include <dji_control.hpp>

// Helpers
#include <dji_linux_helpers.hpp>

void doStep(Vehicle* vehicle);

void getYaw(Vehicle* vehicle, int timeoutParamInMs);

void appendToFile(float yaw, uint32_t timeStamp);

void clearFile();

bool monitoredTakeoff(Vehicle* vehicle, int timeout);

#endif 