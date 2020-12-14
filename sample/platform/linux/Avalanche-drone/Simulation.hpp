#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include "HField.hpp"
#include "V3D.hpp"
#include <time.h>
#include <chrono>


class Simulation
{
private:
    HField hField;
    V3D currentPos;
    double zeroArray[30000] = {0};
    double data[30000];
    std::chrono::steady_clock::time_point sampleClock;
    int tick;
    double sampleRate = 3000000; //set the sampleRate to 3MHz
    double signalFrequency = 457000; //set the signal frequency to 457kHz
public:
    void setupSimulation(double startLong, double startLat, double offsetLong, double offsetLat);
    void setPosition(V3D position);
    double * sample();

    //Constructors
    Simulation(/* args */);
    Simulation(double startLong, double startLat, double offsetLong, double offsetLat);
    ~Simulation();
};


#endif