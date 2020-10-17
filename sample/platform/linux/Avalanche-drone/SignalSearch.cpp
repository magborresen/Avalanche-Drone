


/*! @file mission_sample.cpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  GPS Missions API usage in a Linux environment.
 *  Shows example usage of the Waypoint Missions and Hotpoint Missions through
 * the
 *  Mission Manager API.
 *
 *  @Copyright (c) 2017 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include "SignalSearch.hpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

bool runSignalSearchMission(Vehicle* vehicle, uint8_t maxNumWaypoint, int responseTimeout)
{
    // Waypoint Mission : Initialization
    WayPointInitSettings fdata;
    setWaypointInitDefaults(&fdata);

    fdata.indexNumber = maxNumWaypoint+1; // Sets the max number of waypint + 1 for return to start 

    float32_t fly_alt = 3; //sets the flying altitude to 3 meters

    //initialize the mission manager, Sets fdata as the deafult waypoin setting, sets mission to waypoint mission
    ACK::ErrorCode initAck = vehicle->missionManager->init(DJI_MISSION_TYPE::WAYPOINT, responseTimeout, &fdata);
    if (ACK::getError(initAck))
    {
        ACK::getErrorCodeMessage(initAck, __func__);
    }

    vehicle->missionManager->printInfo();
    std::cout << "Initializing Waypoint Mission..\n";

    // Waypoint Mission: Create Waypoints
    std::vector<WayPointSettings> generatedWaypts = createWaypoints(vehicle, maxNumWaypoint, fly_alt);
    std::cout << "Creating Waypoints..\n";

    // Waypoint Mission: Upload the waypoints
    uploadWaypoints(vehicle, generatedWaypts, responseTimeout);
    std::cout << "Uploading Waypoints..\n";

    // Waypoint Mission: Start
    ACK::ErrorCode startAck = vehicle->missionManager->wpMission->start(responseTimeout);
    if (ACK::getError(startAck))
    {
        ACK::getErrorCodeMessage(initAck, __func__);
    }
    else
    {
        std::cout << "Starting Waypoint Mission.\n";
        sleep(5);
    }
    return true;
    }

void setWaypointDefaults(WayPointSettings* wp)
{
    wp->damping         = 0;
    wp->yaw             = 0;
    wp->gimbalPitch     = 0;
    wp->turnMode        = 0;
    wp->hasAction       = 0;
    wp->actionTimeLimit = 100;
    wp->actionNumber    = 0;
    wp->actionRepeat    = 0;
    for (int i = 0; i < 16; ++i)
    {
        wp->commandList[i]      = 0;
        wp->commandParameter[i] = 0;
    }
}

void setWaypointInitDefaults(WayPointInitSettings* fdata)
{
    fdata->maxVelocity    = 15;
    fdata->idleVelocity   = 5;
    fdata->finishAction   = 0;
    fdata->executiveTimes = 1;
    fdata->yawMode        = 0;
    fdata->traceMode      = 0;
    fdata->RCLostAction   = 1; //TODO: should maybe be zero when testing IRL
    fdata->gimbalPitch    = 0;
    fdata->latitude       = 0;
    fdata->longitude      = 0;
    fdata->altitude       = 0;
}

std::vector<DJI::OSDK::WayPointSettings> createWaypoints(DJI::OSDK::Vehicle* vehicle, int maxNumWaypoint, float32_t fly_alt)
{
    // Create Start Waypoint
    WayPointSettings start_wp;x
    setWaypointDefaults(&start_wp);

    // Global position retrieved via subscription
    Telemetry::TypeMap<TOPIC_GPS_FUSED>::type subscribeGPosition;
    // Global position retrieved via broadcast
    Telemetry::GlobalPosition broadcastGPosition;

    //gets the current GPS position of the drone
    broadcastGPosition = vehicle->broadcast->getGlobalPosition();
    start_wp.latitude  = broadcastGPosition.latitude;
    start_wp.longitude = broadcastGPosition.longitude;
    start_wp.altitude  = start_alt;
    printf("Waypoint created at (LLA): %f \t%f \t%f\n", broadcastGPosition.latitude, broadcastGPosition.longitude, start_alt);
    std::vector<DJI::OSDK::WayPointSettings> wpVector = generateBoustrophedonWaypoints(&start_wp, maxNumWaypoint);
    return wpVector;
}

std::vector<DJI::OSDK::WayPointSettings> generateBoustrophedonWaypoints(WayPointSettings* start_data, int max_wp)
{

    // Let's create a vector to store our waypoints in.
    std::vector<DJI::OSDK::WayPointSettings> wp_list;

    //Calculation for second start point


    // First waypoint
    start_data->index = 0;
    wp_list.push_back(*start_data);

    // Iterative algorithm
    for (int i = 1; i < num_wp; i++)
    {
        WayPointSettings  wp;
        WayPointSettings* prevWp = &wp_list[i - 1];
        setWaypointDefaults(&wp);
        wp.index     = i;
        wp.latitude  = (prevWp->latitude + (increment * cos(i * extAngle)));
        wp.longitude = (prevWp->longitude + (increment * sin(i * extAngle)));
        wp.altitude  = (prevWp->altitude + 1);
        wp_list.push_back(wp);
    }

    // Come back home
    start_data->index = num_wp;
    wp_list.push_back(*start_data);

    return wp_list;
}

void uploadWaypoints(Vehicle* vehicle, std::vector<DJI::OSDK::WayPointSettings>& wp_list, int responseTimeout)
{
    for (std::vector<WayPointSettings>::iterator wp = wp_list.begin(); wp != wp_list.end(); ++wp)
    {
        printf("Waypoint created at (LLA): %f \t%f \t%f\n ", wp->latitude, wp->longitude, wp->altitude);
        ACK::WayPointIndex wpDataACK = vehicle->missionManager->wpMission->uploadIndexData(&(*wp), responseTimeout);
        ACK::getErrorCodeMessage(wpDataACK.ack, __func__);
    }
}

float64_t distanceBetweenGPS(Telemetry::GlobalPosition pos1 , Telemetry::GlobalPosition pos2){ //algoritme stolen from https://www.movable-type.co.uk/scripts/latlong.html
    R = 6373*10^3; //jordens radius
    distanceLongitude = pos1->longitude - pos2->longitude;
    distanceLatidude = pos1->latitude - pos2->latitude;
    float64_t a = (sin(distanceLatidude/2))^2+cos(pos1->latitude)*cos(pos2->latitude)*(sin(distanceLongitud/2))^2;
    float64_t c = 2 * atan2( sqrt(a), sqrt(1-a));
    float64_t distance = R * c;
    return distance;
}
