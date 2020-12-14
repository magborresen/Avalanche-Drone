#ifndef DJIOSDK_FLIGHTCONTROL_HPP
#define DJIOSDK_FLIGHTCONTROL_HPP

// System Includes
#include <cmath>

// DJI OSDK includes
#include "dji_status.hpp"
#include <dji_vehicle.hpp>

// Helpers
#include <dji_linux_helpers.hpp>

bool monitoredTakeoff(Vehicle* vehicle, int timeout);