
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


using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;


#define samples_per_period 20480
//#define N 4096
#define L 4096 //Tager 0,5ms samplingtid at fylde array
int smph_FFT = 1;


struct v2d{
  double x;
  double y;
};

uint16_t ADC_store1[samples_per_period];
uint16_t ADC_store2[samples_per_period];

fftw_complex *FFToutput1;
fftw_complex *FFTinput1;
fftw_complex *FFToutput2;
fftw_complex *FFTinput2;

fftw_plan plan1;
fftw_plan plan2;

HField avaTrans;


int main()
{
	/*
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
	*/
	// FFT setup
	//setup fft

	//setup the H-field simulation
	void setupSimulation();

    V3D test;

    for (int i = 0; i < 20; i++)
    {
        test = avaTrans.getHFieldVector(0,i);
        std::cout << "x: " << test.x << " y: " << test.y << "\n"; 
    }
    for (int i = 0; i < 20; i++)
    {
        test = avaTrans.getHFieldVector(i,0);
        std::cout << "x: " << test.x << " y: " << test.y << "\n"; 
    }
    for (int i = 0; i < 29; i++)
    {
        test = avaTrans.getHFieldVector(i,i);
        std::cout << "x: " << test.x << " y: " << test.y << "\n"; 
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
}

void setupSimulation(){
  	//places virtual avalanchetransmitter
	/*
		Telemetry::GlobalPosition start_pos_1 = vehicle->broadcast->getGlobalPosition();
		calcLatConvertionFactor(start_pos_1.latitude);
		avaTrans.setStartPos(start_pos_1.latitude , start_pos_1.longitude);
	*/
	avaTrans.setStartPos(0 , 0);
	avaTrans.setAvalanchePos(30 , 30);
}

void simulatingFlying(){



}


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