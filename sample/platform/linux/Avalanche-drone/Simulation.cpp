#include "Simulation.hpp"
#include "HField.hpp"
#include <chrono>
#include <thread>

/*
    Method for setting up the simulation give
    @startLong start longitude
    @startLat start latitude
    @offsetLong offset longitude in Meters
    @offsetLat offset latitude in Meters
*/
void Simulation::setupSimulation(double startLong, double startLat, double offsetLong, double offsetLat){
    hField.setStartPos(startLong , startLat);
	hField.setAvalanchePosFromOffset(offsetLong , offsetLat);
    sampleClock = std::chrono::steady_clock::now();
    tick = 0;
}

void Simulation::setPosition(V3D position){
    currentPos = position;
}

double * Simulation::sample(){
    std::chrono::steady_clock::time_point timeStamp = std::chrono::steady_clock::now();

    //checks if 10ms has passed since last time
    if(std::chrono::duration_cast<std::chrono::milliseconds>(sampleClock - timeStamp).count() > 10){
        if(tick < 10){
            //calculate som samples


        }
        else{
            return zeroArray;
        }
    }
}





/*
    Constructor with startLong, startLat, offsetLong, offsetLat
    Calls the setup simulation from start
*/
Simulation::Simulation(double startLong, double startLat, double offsetLong, double offsetLat){
    setupSimulation(startLong , startLat,offsetLong , offsetLat);
    tick = 0;
}

Simulation::LavSimulation(/* args */){
    tick = 0;
}

Simulation::~LavSimulation(){}
