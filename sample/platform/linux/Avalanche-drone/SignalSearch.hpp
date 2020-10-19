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

bool runSignalSearchMission(Vehicle* vehicle, uint8_t maxNumWaypoint, int responseTimeout);

void setWaypointDefaults(WayPointSettings* wp);

void setWaypointInitDefaults(WayPointInitSettings* fdata);

std::vector<DJI::OSDK::WayPointSettings> createWaypoints(DJI::OSDK::Vehicle* vehicle, int maxNumWaypoint, float32_t fly_alt);

void uploadWaypoints(Vehicle* vehicle, std::vector<DJI::OSDK::WayPointSettings>& wp_list, int responseTimeout);

std::vector<DJI::OSDK::WayPointSettings> calculateWaypoints(Telemetry::GlobalPosition startPos1 , Telemetry::GlobalPosition startPos2, int maxWaypoints);

Telemetry::GlobalPosition turningPointCalculator(WayPointSettings pos1 , WayPointSettings pos2, int turnWay);




#endif // SIGNAL_SEARCH_HPP