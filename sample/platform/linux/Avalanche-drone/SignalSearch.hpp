/*
    This header file is for the signal search part
    The header contains the functions needed to complete this part of the system.
*/

#ifndef SIGNAL_SEARCH_HPP
#define SIGNAL_SEARCH_HPP

// System Includes
#include <cmath>
#include <vector>

#include <iostream>
#include <fstream>

// DJI OSDK includes
#include <dji_vehicle.hpp>
#include <dji_telemetry.hpp>

// Helpers
#include <dji_linux_helpers.hpp>

#define SS_PI 3.14159265359;
#define earthRadius 6378137;

float64_t latConvertionFactor    = 0.0000001567848; //deafult latconvertion is for 45 degrees
float64_t longConvertionFactor   = 2*pi/(earthRadius*2*pi); //deafult longitude, doesn't depend on position



bool runSignalSearchMission(Vehicle* vehicle, uint8_t maxNumWaypoint, int responseTimeout);

void setWaypointDefaults(WayPointSettings* wp);

void setWaypointInitDefaults(WayPointInitSettings* fdata);

std::vector<DJI::OSDK::WayPointSettings> createWaypoints(DJI::OSDK::Vehicle* vehicle, int maxNumWaypoint, float32_t fly_alt);

void uploadWaypoints(Vehicle* vehicle, std::vector<DJI::OSDK::WayPointSettings>& wp_list, int responseTimeout);

std::vector<DJI::OSDK::WayPointSettings> calculateWaypoints(Telemetry::GlobalPosition startPos1 , Telemetry::GlobalPosition startPos2, int maxWaypoints);

Telemetry::GlobalPosition turningPointCalculator(WayPointSettings pos1 , WayPointSettings pos2, int turnWay);

void openFile();

void calcLatConvertionFactor();

#endif // SIGNAL_SEARCH_HPP