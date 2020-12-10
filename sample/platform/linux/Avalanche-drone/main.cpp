#include "SignalSearch.hpp"
#include "Kontrol.hpp"
#include "ADC.hpp"
#include "Irr_impl.hpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

int main()
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
  /*
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
  
  
  */
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