/*! @file flight-control/main.cpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  main for Flight Control API usage in a Linux environment.
 *  Provides a number of helpful additions to core API calls,
 *  especially for position control, attitude control, takeoff,
 *  landing.
 *
 *  @Copyright (c) 2016-2017 DJI
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

/*TODO:flight_control_sample will by replace by flight_sample in the future*/
#include "flight_control_sample.hpp"
#include "flight_sample.hpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

int main(int argc, char** argv) {
  // Initialize variables
  int functionTimeout = 1;
  
  uint8_t ctrl_flag_custom = (Control::HORIZONTAL_POSITION | Control::VERTICAL_POSITION  | Control::YAW_ANGLE | Control::HORIZONTAL_BODY | Control::STABLE_ENABLE );
  
  
  // Setup OSDK.
  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle* vehicle = linuxEnvironment.getVehicle();
  if (vehicle == NULL) {
    std::cout << "Vehicle not initialized, exiting.\n";
    return -1;
  }

  // Obtain Control Authority
  vehicle->obtainCtrlAuthority(functionTimeout);

  vehicle->control->velocityAndYawRateCtrl(17, 1, 1, 150);
  monitoredTakeoff(vehicle);
  int i = 0;
  usleep(1000);
  //while(i < 1000) {
  //Control::CtrlData goUp(ctrl_flag_custom, 0, 0, 3, 0);
  //vehicle->control->flightCtrl(goUp);
  //i++;
  //}
  
  i = 0;
  usleep(1000);
  while(i < 1000) {
  Control::CtrlData stepResponse(ctrl_flag_custom, 1, 0, 3, 10);
  vehicle->control->flightCtrl(stepResponse);
  i++;
  }
  monitoredLanding(vehicle);


  return 0;
}
