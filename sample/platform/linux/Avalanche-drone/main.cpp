/*! @file missions/main.cpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  main for GPS Missions API usage in a Linux environment.
 *  Shows example usage of the Waypoint Missions and Hotpoint Missions through
 *  the
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

//#include "SignalSearch.hpp"
//#include <wiringPi.h>
#include "Kontrol.hpp"

//using namespace DJI::OSDK;
//using namespace DJI::OSDK::Telemetry;

int
main()
{
/*  
wiringPiSetup () ;
  pinMode (1, OUTPUT) ;
  // Initialize variables
  int functionTimeout = 1;

  // Setup OSDK.
  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle*   vehicle = linuxEnvironment.getVehicle();
  if (vehicle == NULL)
  {
      std::cout << "Vehicle not initialized, exiting.\n";
      return -1;
  }

  // Obtain Control Authority
  vehicle->obtainCtrlAuthority(functionTimeout);
*/
  // FFT setup
  fftw_complex *FFToutput1;
  fftw_complex *FFTinput1;
  fftw_complex *FFToutput2;
  fftw_complex *FFTinput2;
  double filter_output1[L];
  double filter_output2[L];
  double mag1;
  double mag2;
  double phase1;
  double phase2;
  double res;
  FFTinput1 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);
  FFToutput1 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);
  FFTinput2 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);
  FFToutput2 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);
  fftw_plan plan1 = fftw_plan_dft_1d(N,
													  FFTinput1,
													  FFToutput1,
													  FFTW_FORWARD,
													  FFTW_ESTIMATE);

  fftw_plan plan2 = fftw_plan_dft_1d(N,
													  FFTinput2,
													  FFToutput2,
													  FFTW_FORWARD,
													  FFTW_ESTIMATE);
													  
													  
//  runSignalSearchMission(vehicle, 8 , 1);
  
  // Full simulation from filteroutput to angle , alpha should equal SIMANGLE
  input_sim(filter_output1, filter_output2);
  cast2complex(filter_output1, FFTinput1);
  cast2complex(filter_output2, FFTinput2);
  do_FFT(&plan1, FFToutput1, &mag1, &phase1);
  do_FFT(&plan2, FFToutput2, &mag2, &phase2);
  
  res = calc_Angle(mag1, mag2, phase1, phase2);
  
  
  
  return 0;
}


/* OLD STUFF
  // Setup variables for use
  uint8_t wayptPolygonSides;
  int     hotptInitRadius;
  int     responseTimeout = 1;

  // Display interactive prompt
  std::cout
    << "| Available commands:                                            |"
    << std::endl;
  std::cout
    << "| [a] Waypoint Mission                                           |"
    << std::endl;
  std::cout
    << "| [b] Hotpoint Mission                                           |"
    << std::endl;
  char inputChar;
  std::cin >> inputChar;
  switch (inputChar)
  {
    case 'a':
      // Waypoint call
      wayptPolygonSides = 6;
      runWaypointMission(vehicle, wayptPolygonSides, responseTimeout);
      break;
    case 'b':
      hotptInitRadius = 10;
      runHotpointMission(vehicle, hotptInitRadius, responseTimeout);
      break;
    default:
      break;
  }
*/