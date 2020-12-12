#include "SignalSearch.hpp"
#include "Kontrol.hpp"
#include "ADC.hpp"
#include "IIRFilter.hpp"
#include <vector>
#include <iostream>
#include <fstream>
#include <thread>
#include <mutex>
#include <chrono>

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

std::mutex mtx_adc_store;
std::mutex mtx_fft_signal;

int smph_FFT = 1;

uint16_t ADC_store1[L];
uint16_t ADC_store2[L];

fftw_complex *FFToutput1;
fftw_complex *FFTinput1;
fftw_complex *FFToutput2;
fftw_complex *FFTinput2;

fftw_plan plan1;
fftw_plan plan2;


void thr_adc_read(){
  while(true){
    static int i_adc = 0;
    vector<uint16_t> ADC_read;

    ADC_read = readADC();
    mtx_adc_store.lock();
    ADC_store1[i_adc] = ADC_read[0];
    ADC_store2[i_adc] = ADC_read[1];
    mtx_adc_store.unlock();
    if(i_adc >= L){
      i_adc = 0;
    }
    else{
      i_adc++;
    }
  }
}

void thr_filter(){  
  while(true){
    static int counter = 0;
    static IIRFilter filter1;
    static IIRFilter filter2;
    mtx_adc_store.lock();
    FFTinput1[i][REAL] = filter1.filter(ADC_store1[counter]);
    FFTinput2[i][REAL] = filter2.filter(ADC_store2[counter]);
    mtx_adc_store.unlock();
    if(counter >= L){
      counter = 0;
      smph_FFT = 1;
    }
    else{
      counter++;
    }
  }
}

void thr_fft(){
  while(true){
    static double mag1, mag2, phase1, phase2;
    if(smph_FFT == 1){
      do_FFT(&plan1, FFToutput1, &mag1, &phase1);
      do_FFT(&plan2, FFToutput2, &mag2, &phase2);
      cout << "Mag1: " << mag1 << "\n Mag2: " << mag2 << "\n phase1: " << phase1 << "\n phase2: " << phase2 << "\n";
      smph_FFT = 0;
    }
  }
}


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
  double filter_output1[L];
  double filter_output2[L];
  double mag1;
  double mag2;
  double phase1;
  double phase2;
  double res;
  //setup fft
  FFTinput1 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);
  FFToutput1 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);
  FFTinput2 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);
  FFToutput2 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);
  plan1 = fftw_plan_dft_1d(N, FFTinput1, FFToutput1, FFTW_FORWARD, FFTW_ESTIMATE);
  plan2 = fftw_plan_dft_1d(N, FFTinput2, FFToutput2, FFTW_FORWARD, FFTW_ESTIMATE);

  // ADC setup
  startADCSPI();
  
  //setup filter
  //Make 2 filter objects so that the stored w in each filter is preserved and do not interfer with the other. 

  //thread setup
  std::thread tADC(thr_adc_read);
  std::thread tFilter(thr_filter);
  std::thread tFFT(thr_fft);

  tADC.join();
  tFilter.join();
  tFFT.join();

  //make thread to read ADCs
  std::fstream file;
  file.open("TestData.txt", std::fstream::out | std::fstream::trunc);

  return 0;
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