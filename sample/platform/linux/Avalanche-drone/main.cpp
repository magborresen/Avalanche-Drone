
//#include "Kontrol.hpp"
#include "ADC.hpp"
#include "IIRFilter.hpp"
#include <fftw3.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <chrono>
#include <cmath>
#include "HField.hpp"
#include "SignalSearch.hpp"
#include "V3D.hpp"
#include "FlightController.hpp"

// DJI OSDK includes
#include "dji_status.hpp"
#include <dji_vehicle.hpp>
#include "dji_control.hpp"

// Helpers
#include <dji_linux_helpers.hpp>

#include "Simulation.hpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;


#define samples_per_period 20480
//#define N 4096
#define L 4096 //Tager 0,5ms samplingtid at fylde array

uint16_t ADC_store1[samples_per_period];
uint16_t ADC_store2[samples_per_period];


fftw_complex *FFToutput1;
fftw_complex *FFTinput1;
fftw_complex *FFToutput2;
fftw_complex *FFTinput2;

fftw_plan plan1;
fftw_plan plan2;

Simulation avaTransSim;


int tick = 0;


int main(int argc, char** argv)
{
	int functionTimeout = 1;
    // Setup OSDK.
    LinuxSetup linuxEnvironment(argc, argv);
    Vehicle*   vehicle = linuxEnvironment.getVehicle();
    if (vehicle == NULL)
    {
        std::cout << "Vehicle not initialized, exiting.\n";
        return -1;
    }
    //  runSignalSearchMission(vehicle, 8 , 1);
    // Obtain Control Authority
    vehicle->obtainCtrlAuthority(functionTimeout);
	monitoredTakeoff(vehicle);


    /*
    From doc:
        HORIZONTAL_VELOCITY - Set the control-mode to control horizontal vehicle velocities.
        VERTICAL_POSITION  - Set the control-mode to control the height of UAV
        HORIZONTAL_BODY - Set the x-y of body frame as the horizontal frame (FRU) 
        YAW_ANGLE - Set the control-mode to control yaw angle.
        STABLE_ENABLE - Enable the stable mode 
    */
    uint8_t ctrl_flag_costum = (Control::HORIZONTAL_VELOCITY | Control::VERTICAL_POSITION  | Control::YAW_ANGLE | Control::HORIZONTAL_BODY | Control::STABLE_ENABLE )
	
    //setup the H-field simulation
    avaTransSim.setupSimulation(0,0,30,30);
    Telemetry::GlobalPosition currentBroadcastGP;
    while(true){
        Control::CtrlData custumData(ctrl_flag_costum, 1 , 0, 2, 5);
        vehicle->control->flightCtrl(custumData);
        currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
        std::cout << "X: " << currentBroadcastGP.latitude << " Y: " << currentBroadcastGP.longitude << "\n";
        usleep(20 * 1000);
    }
    
    /*
    V3D posNow(0,0,0);
    V3D velNow(1,0,0);
    dataPack recivedSignal;
    auto sampleClock = std::chrono::high_resolution_clock::now();
    auto timeNow = std::chrono::high_resolution_clock::now();
    int counter = 0;

    while(counter < 2){
        timeNow = std::chrono::high_resolution_clock::now();    
        auto timediff = timeNow-sampleClock;
        auto timediffMS = std::chrono::duration_cast<std::chrono::milliseconds>(timediff).count();

        if(timediffMS >= 10){
            avaTransSim.setPosition(posNow);
            avaTransSim.calculateErrorAngleAndSize(velNow);
            recivedSignal = avaTransSim.sample(counter);
            sampleClock = std::chrono::high_resolution_clock::now();
            counter++;
            for (int i = 0; i < 10; i++)
            {
                std::cout << "A1:" << recivedSignal.A1[i] << "  A2: " << recivedSignal.A2[i] << "\n";
            }
        }
    }
    */
}



/*
	FFTinput1 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);
	FFToutput1 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);
	FFTinput2 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);
	FFToutput2 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);
	plan1 = fftw_plan_dft_1d(N, FFTinput1, FFToutput1, FFTW_FORWARD, FFTW_ESTIMATE);
	plan2 = fftw_plan_dft_1d(N, FFTinput2, FFToutput2, FFTW_FORWARD, FFTW_ESTIMATE);

	// ADC setup
	startADCSPI();
	while(true){
		vector<uint16_t> sampledData = readADC(samples_per_period);
	}

	std::fstream file;
	file.open("TestData.txt", std::fstream::out | std::fstream::trunc);
	file.close();
	return 0;
*/

 /*
  for(int j = 0; j < 20 ; j++){
      //read L number of datapoints from ADC
    for(int i = 0; i < L ; i++){
      ADC_read = readADC();
      ADC_store1[i] = ADC_read[0];
      ADC_store2[i] = ADC_read[1];
    }

    //Filter the read data and move into FFT array
    for(int i = 0; i < L ; i++){
      FFTinput1[i][REAL] = filter1.filter(ADC_store1[i]);
      FFTinput1[i][IMAG] = 0;
      FFTinput2[i][REAL] = filter2.filter(ADC_store2[i]);
      FFTinput2[i][IMAG] = 0;
    }  

    filter1.resetFilter();
    filter2.resetFilter();
    
    do_FFT(&plan1, FFToutput1, &mag1, &phase1);
    do_FFT(&plan2, FFToutput2, &mag2, &phase2);
    cout << "A1 = " << mag1 << "\n";
    cout << "A2 = " << mag2 << "\n";
    cout << "Phase1 = " << phase1 << "\n";
    cout << "Mag1 = " << phase2 << "\n";
    file << j << "," << mag1 << "," << mag2 << "," << phase1 << "," << phase2 << "\n";
  
  } 
  */

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