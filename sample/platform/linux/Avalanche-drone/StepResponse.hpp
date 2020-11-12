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

#define C_EARTH (double)6378137.0
#define DEG2RAD 0.01745329252

void doStep(Vehicle* vehicle);

void getYaw(Vehicle* vehicle, int timeoutParamInMs);

void appendToFile(double yaw, uint32_t timeStamp);

void clearFile();

bool monitoredTakeoff(Vehicle* vehicle, int timeout = 1);

bool moveByPositionOffset(DJI::OSDK::Vehicle *vehicle, float xOffsetDesired, float yOffsetDesired, float zOffsetDesired, float yawDesired, float posThresholdInM = 0.5,float yawThresholdInDeg = 1.0);
						  
void localOffsetFromGpsOffset(DJI::OSDK::Vehicle*             vehicle,
                              DJI::OSDK::Telemetry::Vector3f& deltaNed,
                              void* target, void* origin);

DJI::OSDK::Telemetry::Vector3f toEulerAngle(void* quaternionData);
bool startGlobalPositionBroadcast(DJI::OSDK::Vehicle* vehicle);

#endif 