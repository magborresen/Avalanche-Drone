
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

// DJI OSDK includesw
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
#define fftbin 624
#define FFTSize 4096
#define REAL 0
#define IMAG 1
#define DEG2RAD 0.01745329252

uint16_t ADC_store1[samples_per_period];
uint16_t ADC_store2[samples_per_period];


fftw_complex *FFToutput;
fftw_complex *FFTinput;
fftw_plan plan;



Simulation avaTransSim;

int tick = 0;

Telemetry::Vector3f toEulerAngle(void* quaternionData){
    Telemetry::Vector3f    ans;
    Telemetry::Quaternion* quaternion = (Telemetry::Quaternion*)quaternionData;

    double q2sqr = quaternion->q2 * quaternion->q2;
    double t0    = -2.0 * (q2sqr + quaternion->q3 * quaternion->q3) + 1.0;
    double t1 =
        +2.0 * (quaternion->q1 * quaternion->q2 + quaternion->q0 * quaternion->q3);
    double t2 =
        -2.0 * (quaternion->q1 * quaternion->q3 - quaternion->q0 * quaternion->q2);
    double t3 =
        +2.0 * (quaternion->q2 * quaternion->q3 + quaternion->q0 * quaternion->q1);
    double t4 = -2.0 * (quaternion->q1 * quaternion->q1 + q2sqr) + 1.0;

    t2 = (t2 > 1.0) ? 1.0 : t2;
    t2 = (t2 < -1.0) ? -1.0 : t2;

    ans.x = asin(t2);
    ans.y = atan2(t3, t4);
    ans.z = atan2(t1, t0);

    return ans;
}


//Move a double into the FFTinput array
void moveToFFT(double signalToMove[], int offset_N){
    for (int i = 0; i < FFTSize; i++)
    {
        FFTinput[i][REAL] = signalToMove[i+offset_N*FFTSize];
        FFTinput[i][IMAG] = 0;
    }
}

/*
    retReal return real values
    retImag return Imag values
*/
void doTheFFT(double signalToFFT[], double retReal[], double retImag[]){
    int numberOfFFTs = samples_per_period/FFTSize;
    int reminderOfFFT = samples_per_period % FFTSize;
    for (int i = 0; i < numberOfFFTs; i++)
    {
        moveToFFT(signalToFFT, i);
        fftw_execute(plan);

        retReal[i] = FFToutput[fftbin][REAL];
        retImag[i] = FFToutput[fftbin][IMAG];
    }
    return;
}

/*
    Calculate the Magnitude from the fft take the average of 5 times fft samples
*/
double getFFTMagnitudeMean(double realVal[], double imagVal[]){
    double sum = 0;
    for (int i = 0; i < 5; i++)
    {
        sum += std::sqrt(std::pow(realVal[i],2) + std::pow(imagVal[i],2));
    }
    return sum/5;
}

/*
    Calculate the phase from the fft take the average of 5 times fft samples
*/
double getFFTAngleMean(double realVal[], double imagVal[]){
    double sum = 0;
    for (int i = 0; i < 5; i++)
    {
        sum += std::atan2(imagVal[i] , realVal[i]);
    }
    return sum/5;
}


/*
    Calculate the error angle from magnitude from the two sinals and there angle
    The output is in degrees and not radians
*/
double calculateErrorAngle(double Mag1, double Mag2, double ang1, double ang2){
    double H = sqrt(pow(Mag1,2)+pow(Mag2,2)); // calculating the length of the H-field
	double alpha = acos(Mag1/H)*180.0/PI; // the angle we want to find alpha = acos( A / |H| ) from A = cos(alhpa)* |H|
	if(ang1 - ang2 ==0)
    {
        alpha = alpha*-1;
    }
	return alpha;
}


int main(int argc, char** argv)
{
    //Setup FFT
    FFTinput = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * FFTSize);
	FFToutput = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * FFTSize);
    plan = fftw_plan_dft_1d(FFTSize, FFTinput, FFToutput, FFTW_FORWARD, FFTW_ESTIMATE);

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
    
    /*
    auto stampClockSample = std::chrono::high_resolution_clock::now();
    auto stampClockControl = std::chrono::high_resolution_clock::now();
    auto timeNow = std::chrono::high_resolution_clock::now();
    */
   
    int counter = 0;
    int ct = 0;
    dataPack recivedSignal;
    double yaw = 0;
    double fftA1Real[5];
    double fftA1Imag[5];
    double fftA2Real[5];
    double fftA2Imag[5];

    double goalYaw = 0;
    int tick = 0;
    /*
      Starting main loop
    */
    while(true){
        Control::CtrlData custumData(ctrl_flag_costum, 1 , 0, 2, goalYaw);
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

        doTheFFT(recivedSignal.A1, fftA1Real , fftA1Imag);
        doTheFFT(recivedSignal.A2, fftA2Real , fftA2Imag);

        double A1meanMag = getFFTMagnitudeMean(fftA1Real,fftA1Imag);
        double A2meanMag = getFFTMagnitudeMean(fftA2Real,fftA2Imag);
        double A1meanAngle = getFFTAngleMean(fftA1Real, fftA1Imag);
        double A2meanAngle = getFFTAngleMean(fftA2Real, fftA2Imag);
        

        //Quaternion show der ikke er nogle der forstår tyv stjålet fra DJI
        Telemetry::Quaternion quat;
        quat = vehicle->broadcast->getQuaternion();

        if(A1meanMag > 0 || A2meanMag > 0){
            tick++;
        }
        else{
            tick = 0;
        }

        if(tick > 3){
            double errorAngle = calculateErrorAngle(A1meanMag,A2meanMag,A1meanAngle,A2meanAngle);
            tick = 0;
            double yawInRad = toEulerAngle((static_cast<void*>(&quat))).z / DEG2RAD;
            //set new goalyaw
            goalYaw = yawInRad+errorAngle;
            std::cout << "Goal yaw: " << goalYaw << "\n";
        }
    }
    
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