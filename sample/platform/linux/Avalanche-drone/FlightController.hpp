#ifndef FLIGHTCONTROL_HPP
#define FLIGHTCONTROL_HPP

// System Includes
#include <cmath>

// DJI OSDK includes
#include "dji_status.hpp"
#include <dji_vehicle.hpp>

// Helpers
#include <dji_linux_helpers.hpp>

bool monitoredTakeoff(Vehicle* vehicle, int timeout = 1);
bool monitoredLanding(Vehicle* vehicle, int timeout = 1);
#endif