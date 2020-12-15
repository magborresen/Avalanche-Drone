
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
#include "fft_test.hpp"

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

#define FFTSize 4096

Simulation avaTransSim;

int tick = 0;


int main(int argc, char** argv)
{
    fftTest()

    /*
    //Setup FFT
    FFTinput1 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * FFTSize);
	FFToutput1 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * FFTSize);
	FFTinput2 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * FFTSize);
	FFToutput2 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * FFTSize);

    plan1 = fftw_plan_dft_1d(FFTSize, FFTinput1, FFToutput1, FFTW_FORWARD, FFTW_ESTIMATE);
	plan2 = fftw_plan_dft_1d(FFTSize, FFTinput2, FFToutput2, FFTW_FORWARD, FFTW_ESTIMATE);


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
    */
    /*
    From doc:
        HORIZONTAL_VELOCITY - Set the control-mode to control horizontal vehicle velocities.
        VERTICAL_POSITION  - Set the control-mode to control the height of UAV
        HORIZONTAL_BODY - Set the x-y of body frame as the horizontal frame (FRU) 
        YAW_ANGLE - Set the control-mode to control yaw angle.
        STABLE_ENABLE - Enable the stable mode 
    */
   /*
    uint8_t ctrl_flag_costum = (Control::HORIZONTAL_VELOCITY | Control::VERTICAL_POSITION  | Control::YAW_ANGLE | Control::HORIZONTAL_BODY | Control::STABLE_ENABLE );
	
    //setup the H-field simulation
    Telemetry::GlobalPosition currentBroadcastGP;
    currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
    avaTransSim.setupSimulation(currentBroadcastGP.longitude,currentBroadcastGP.latitude,30,30);

    Control::CtrlData custumData(ctrl_flag_costum, 1 , 0, 2, 20);

    vehicle->control->flightCtrl(custumData);
    usleep(1000*20);
    currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
    std::cout << "X: " << currentBroadcastGP.latitude << " Y: " << currentBroadcastGP.longitude << "\n";

    Telemetry::Vector3f currentVel;
    currentVel = vehicle->broadcast->getVelocity();
    V3D posNow(currentBroadcastGP.longitude,currentBroadcastGP.latitude,0);
    V3D velNow(currentVel.x,currentVel.y,0);
    */
    /*
    auto stampClockSample = std::chrono::high_resolution_clock::now();
    auto stampClockControl = std::chrono::high_resolution_clock::now();
    auto timeNow = std::chrono::high_resolution_clock::now();
    *//*
    int counter = 0;
    int ct = 0;
    dataPack recivedSignal;
    double yaw = 0;
    */
    /*
      Starting main loop
    */
   /*
    while(true){
        Control::CtrlData custumData(ctrl_flag_costum, 1 , 0, 2, yaw);
        vehicle->control->flightCtrl(custumData);
        usleep(1000*20);

        currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
        currentVel = vehicle->broadcast->getVelocity();
        posNow.x = currentBroadcastGP.longitude;
        posNow.y = currentBroadcastGP.latitude;
        velNow.x = currentVel.x;
        velNow.y = currentVel.y;
        //std::cout << "vel X: " <<  velNow.x << " Y: " << velNow.y << "\n";

        avaTransSim.setPosition(posNow);
        avaTransSim.calculateErrorAngleAndSize(velNow);

        if(ct < 5){
            recivedSignal = avaTransSim.sample(1);
        }
        else{
            recivedSignal = avaTransSim.sample(0);
        }
        ct++;
        if(ct >=30){
            ct = 0;
        }
    }
    */
}


	
	
	// ADC setup
	startADCSPI();
	while(true){
		vector<uint16_t> sampledData = readADC(samples_per_period);
	}

	std::fstream file;
	file.open("TestData.txt", std::fstream::out | std::fstream::trunc);
	file.close();
	return 0;


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