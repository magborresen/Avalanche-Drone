


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

const int earthRadius = 6378137;
const float64_t pi = 3.14159265359;

std::fstream myfile;
std::string filename = "codeLog.txt";

float64_t latConvertionFactor    = 0.0000001567848; //deafult latconvertion is for 45 degrees
float64_t longConvertionFactor   = 2*pi/(earthRadius*2*pi); //deafult longitude, doesn't depend on position

//function for clearing log file
void openFile()
{
    myfile.open(filename,  std::ofstream::out | std::ofstream::trunc);
    myfile.close();
}

void calcLatConvertionFactor(float64_t lat)
{
    float64_t smallCircleRadius = earthRadius * std::sin(pi/2 - lat); //calculate circle of sphere radius
    float64_t smallCircleCircumstance = 2 * smallCircleRadius * pi; //calculate circle of sphere circumstance
    float64_t latRadPerMeter = (2*pi) / smallCircleCircumstance; //calculate radian per meter
    longConvertionFactor = latRadPerMeter;
}

bool runSignalSearchMission(Vehicle* vehicle, uint8_t maxNumWaypoint, int responseTimeout)
{
    openFile();
    // Waypoint Mission : Initialization
    WayPointInitSettings fdata;
    setWaypointInitDefaults(&fdata);
    fdata.indexNumber = maxNumWaypoint; // Sets the max number of waypint + 1 for return to start 

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
    wp->altitude        = 3;
    for (int i = 0; i < 16; ++i)
    {
        wp->commandList[i]      = 0;
        wp->commandParameter[i] = 0;
    }
}

void setWaypointInitDefaults(WayPointInitSettings* fdata)
{
    fdata->maxVelocity    = 15;
    fdata->idleVelocity   = 10;
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
    // Global position retrieved via broadcast
    Telemetry::GlobalPosition start_pos_1;
    Telemetry::GlobalPosition start_pos_2;
    //gets the current GPS position of the drone
    start_pos_1 = vehicle->broadcast->getGlobalPosition();
    start_pos_2 = start_pos_1;
    start_pos_2.longitude = start_pos_2.longitude + (longConvertionFactor*100*3.14592/180); //move second starting point 100meter in longitude 
    calcLatConvertionFactor(start_pos_1.latitude);
    std::vector<DJI::OSDK::WayPointSettings> wpVector = calculateWaypoints(start_pos_1, start_pos_2, maxNumWaypoint);
    return wpVector;
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

std::vector<DJI::OSDK::WayPointSettings> calculateWaypoints(Telemetry::GlobalPosition startPos1 , Telemetry::GlobalPosition startPos2, int maxWaypoints)
{
    float64_t v_start[2]; //v vector 
    std::vector<DJI::OSDK::WayPointSettings> wp_list;
    WayPointSettings prev_wp;
    WayPointSettings old_prev_wp;
    v_start[0] = startPos2.latitude - startPos1.latitude; //calculate v latitude
    v_start[1] = startPos2.longitude - startPos1.longitude; //calculate v longitude
    //load wp_list with the first two points
    WayPointSettings  wp;
    setWaypointDefaults(&wp);
    wp.index = 0;
    wp.latitude = startPos1.latitude;
    wp.longitude = startPos1.longitude;
    wp_list.push_back(wp);
    std::cout << "Point " << 0 << " - lat: " << wp.latitude << " long: " << wp. longitude << "\n";
    prev_wp = wp;
    setWaypointDefaults(&wp);
    wp.index = 1;
    wp.latitude = startPos2.latitude;
    wp.longitude = startPos2.longitude;
    old_prev_wp = prev_wp;
    prev_wp = wp;
    wp_list.push_back(wp);
    std::cout << "Point " << 1 << " - lat: " << wp.latitude << " long: " << wp. longitude << "\n";

    int state = 0;

    for(int i = 2; i < maxWaypoints ; i++){
        if(state == 0){
            Telemetry::GlobalPosition tp1 = turningPointCalculator(old_prev_wp, prev_wp, 0);
            setWaypointDefaults(&wp);
            wp.index = i;
            wp.latitude = tp1.latitude;
            wp.longitude = tp1.longitude;
            old_prev_wp = prev_wp;
            prev_wp = wp;
            wp_list.push_back(wp);
            state = 1;
        }
        else if(state == 1){
            setWaypointDefaults(&wp);
            wp.index = i;
            wp.latitude = prev_wp.latitude + (-1*v_start[0]);
            wp.longitude = prev_wp.longitude + (-1*v_start[1]);
            old_prev_wp = prev_wp;
            prev_wp = wp;
            wp_list.push_back(wp);
            state = 2;
        }
        else if(state == 2){
            Telemetry::GlobalPosition tp2 = turningPointCalculator(old_prev_wp, prev_wp, 1);
            wp.index = i;
            wp.latitude = tp2.latitude;
            wp.longitude = tp2.longitude;
            old_prev_wp = prev_wp;
            prev_wp = wp;
            wp_list.push_back(wp);
            state = 3;
        }
        else if(state == 3){
            setWaypointDefaults(&wp);
            wp.index = i;
            wp.latitude = prev_wp.latitude + v_start[0];
            wp.longitude = prev_wp.longitude + v_start[1];
            old_prev_wp = prev_wp;
            prev_wp = wp;
            wp_list.push_back(wp);
            state = 0;
        }
        std::cout << "Point " << i << " - lat: " << wp.latitude << " long: " << wp. longitude << "\n";
    }
    return wp_list;
}

Telemetry::GlobalPosition turningPointCalculator(WayPointSettings pos1 , WayPointSettings pos2, int turnWay) //turnway:  0 = ccw, 1 = cw, 
{
    
    float64_t v[2]; //v vector 
    float64_t v_XY[2]; //v in normal coordinates
    float64_t vE_XY[2]; //v vector as a unit vector
    float64_t nD_XY[2]; //new direction vector as a unit vector
    float64_t nD[2];
    
    v[0] = pos2.latitude - pos1.latitude; //calculate v latitude
    v[1] = pos2.longitude - pos1.longitude; //calculate v longitude
    
    //Shift vector to meter coordinate system
    v_XY[0] = v[0] / latConvertionFactor;
    v_XY[1] = v[1] / longConvertionFactor;
    
    float64_t vD = sqrt(v_XY[0]*v_XY[0]+v_XY[1]*v_XY[1]); //calculate the distance between the two points

    //calculate unit vector
    vE_XY[0] = v_XY[0] / vD; 
    vE_XY[1] = v_XY[1] / vD; 
    myfile.open(filename, std::fstream::out);
    myfile << "v[0]: " << v[0] << " v[1]: " << v[1] << "\n";
    myfile << "v_XY[0]: " << v_XY[0] << " v_XY[1]: " << v_XY[1] << "\n";
    myfile << "vD" << vD << "\n";
    
    /*
        Using rotation matrix th = theta
              [ cos(th)    -sin(th) ]
        Rot = [ sin(th)     cos(th) ]
                
        For making a 90 degrees turn th = 90
              [ 0   -1 ]
        rot = [ 1    0 ]
        The matrix computation therefore yields:
                  [ 0   -1 ]   [a]    [ -b ]
        rot * v = [ 1    0 ] * [b] =  [ a  ]
    */

    //calculate the new direction unit vector from rotation  
    if(turnWay == 0) //rotate counter clockwise
    {
        nD_XY[0] = -1*vE_XY[1];
        nD_XY[1] = vE_XY[0];
    }
    else if(turnWay == 1)
    {
        nD_XY[0] = vE_XY[1];
        nD_XY[1] = -1*vE_XY[0];
    }

    myfile << "nD_XY[0]: " << nD_XY[0] << " nD_XY[1]: " << nD_XY[1] << "\n";
    //calculate new vector
    nD_XY[0] = nD_XY[0]*70;
    nD_XY[1] = nD_XY[1]*70;
    myfile << "nD_XY[0]: " << nD_XY[0] << " nD_XY[1]: " << nD_XY[1] << "\n";
    //shift backto long/lat coordinates
    nD[0] = nD_XY[0] * latConvertionFactor;
    nD[1] = nD_XY[1] * longConvertionFactor;

    myfile << "nD[0]: " << nD[0] << " nD[1]: " << nD[1] << "\n";

    Telemetry::GlobalPosition pos3;
    pos3.latitude = pos2.latitude + nD[0];
    pos3.longitude = pos2.longitude + nD[1];

    myfile << "New lat: " << pos3.latitude << " long: " << pos3.longitude << "\n";

    myfile.close();
    return pos3;
}

