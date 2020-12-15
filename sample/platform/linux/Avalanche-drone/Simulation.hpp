#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include "HField.hpp"
#include "V3D.hpp"
#include <time.h>
#include <chrono>

#define amplifierGain 50000 //50000 times gain from analog circuit

struct dataPack
{
    double A1[60000] = {0};
    double A2[60000] = {0};
};

class Simulation
{
private:
    HField hField;
    V3D currentPos;
    V3D curretnHVector;
    double zeroArray[60000] = {0};
    double signal[60000];
    //std::chrono::steady_clock::time_point sampleClock;
    int tick;
    double sampleRate = 3000000; //set the sampleRate to 3MHz
    double signalFrequency = 457000; //set the signal frequency to 457kHz
    double errorAngle;
    double antenna_main_Scale; //main antenna scale factor main antenna is the antenna pointing in the direction of hte flight
    double antenna_second_Scale; //second antenna scale factor antenna orthogonal to flight direction
    double HFieldSize;
    void calculateAntennaSignalStrenght();
public:
    void calculateErrorAngleAndSize(V3D droneVelocityVector);
    void setupSimulation(double startLong, double startLat, double offsetLong, double offsetLat);
    void setPosition(V3D position);
    void printCurrentHVector();
    V3D getCurrentHVector();

    dataPack sample(int flag);

    //Constructors
    Simulation(/* args */);
    Simulation(double startLong, double startLat, double offsetLong, double offsetLat);
    ~Simulation();
};





#endif