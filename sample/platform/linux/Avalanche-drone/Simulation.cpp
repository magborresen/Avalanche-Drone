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
    tick = 0;
}

void Simulation::setPosition(V3D position){
    currentPos = position;
}

dataPack Simulation::sample(int flag){
    calculateAntennaSignalStrenght();
    dataPack returnPack;
    //checks if 10ms has passed since last time
    if(flag = 1){
        //calculate samples
        for (int i = 0; i < 30000; i++)
        {
            returnPack.A1[i] = std::sin(sampleRate*signalFrequency*2*PI*i);
            //Make the second antenna sinus lag if the H field is 2 the right of the antenna.
            if(errorAngle > 0){
                returnPack.A2[i] = std::sin(sampleRate*signalFrequency*2*PI*i) * antenna_main_Scale;
            }
            else{
                returnPack.A2[i] = std::sin(sampleRate*signalFrequency*2*PI*i - PI/4) * antenna_second_Scale;
            }   
        }
        return returnPack;
    }
    else{
        return returnPack;
    }
}




void Simulation::calculateErrorAngleAndSize(V3D droneVelocityVector){
    V3D hvector = hField.getHFieldVector(currentPos.x , currentPos.y);

    //Calculate lenght of droneVelocityVector vector and Hvector
    double lVV = std::sqrt(std::pow(droneVelocityVector.x,2) + std::pow(droneVelocityVector.y,2));
    double lHV = std::sqrt(std::pow(hvector.x,2) + std::pow(hvector.y,2));

    double crossPz = (droneVelocityVector.x * hvector.y) - (droneVelocityVector.y*hvector.x);
    double dotProduct = hvector.x * droneVelocityVector.x + hvector.y * droneVelocityVector.y;

    if(crossPz > 0){
        errorAngle = std::acos(dotProduct/(lVV*lHV));
    }
    else{
        errorAngle = -std::acos(dotProduct/(lVV*lHV));
    }

    //calculate field size as sqrt(x^2 + y^2)
    HFieldSize = std::sqrt(std::pow(hvector.x,2) + std::pow(hvector.y,2));
}


/*
    Calculate the antenna signal strenght from the H-field
    Rember to call calculateErrorAngleAndSize before calling this function
    As it depends on the HFieldSize and errorAngle
*/
void Simulation::calculateAntennaSignalStrenght(){
    //calculate scale factor from A1 = |H|*cos(alpha)
    antenna_main_Scale = HFieldSize * std::cos(errorAngle);

    //calculate scale factor from A2 = |H|*cos(90-alpha)
    antenna_second_Scale = HFieldSize * std::cos(90-errorAngle); 
}

/*
    Constructor with startLong, startLat, offsetLong, offsetLat
    Calls the setup simulation from start
*/
Simulation::Simulation(double startLong, double startLat, double offsetLong, double offsetLat){
    setupSimulation(startLong , startLat,offsetLong , offsetLat);
    tick = 0;
}

Simulation::Simulation(/* args */){
    tick = 0;
}

Simulation::~Simulation(){}