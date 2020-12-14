#include "HField.hpp"
#include "SignalSearch.hpp"
#include "V3D.hpp"
#include <iostream>

HField::HField(/* args */)
{
    //initialize phi
    double j = -PI/2.0;
    int k = 0;
    while(j < 3*PI/2){
        phi[k] = j;
        j +=2*PI/(HFIELD_HPP_N-1);
        std::cout << "Phi(" << k << ") = " << phi[k] << "\n";
        k++;
    }
    phi[k] = j;
    std::cout << "Phi(" << k << ") = " << phi[k] << "\n";

    for (int i = 0; i < HFIELD_HPP_N; i++)
    {
        Xc[i] = std::cos(phi[i]) * Ra;
        std::cout << "Xc: " << Xc[i] << "\n";
    }
    for (int i = 0; i < HFIELD_HPP_N; i++)
    {
        Yc[i] = std::sin(phi[i]) * Ra;
        std::cout << "Yc: " << Yc[i] << "\n";
    }

    //reset startPos to 0
    startPos.x = 0;
    startPos.y = 0;
    startPos.z = 0;
}

HField::~HField()
{
}


/*
    calculate R-vector from the coil(X-Y plane)to Y-Z plane where we are 
    interested to find the magnetic field and also the dl-vector along the
    coil where current is flowing
    (note that R is the position vector pointing from coil (X-Y plane) to
    the point where we need the magnetic field (in this case Y-Z plane).)
    dl is the current element vector which will make up the coil
*/
void HField::calculate_R_vector(double y , double z){
    std::cout << "R_vector \n";
    for (int i = 0; i < (HFIELD_HPP_N-2); i++)
    {
        Rx[i] = -0.5 * (Xc[i] + Xc[i+1]);
        Ry[i] = (y - ( 0.5 * (Yc[i]+ Yc[i+1]) ));
        Rz[i] = z;
        dlx[i] = Xc[i+1]-Xc[i];
        dly[i] = Yc[i+1]-Yc[i];
        std::cout << "Rx,Ry,Rz: " << Rx[i] << "," <<  Ry[i] << "," <<  Rz[i];
        std::cout << "    dlx,dly: " << dlx[i] << "," << dly[i] << "\n" ;
    }
    //set the last element
    Rx[HFIELD_HPP_N-1] = -0.5 * (Xc[HFIELD_HPP_N]-1 + Xc[0]);
    Ry[HFIELD_HPP_N-1] = (y - ( 0.5 * (Yc[HFIELD_HPP_N-1]+ Yc[0]) ));
    Rz[HFIELD_HPP_N-1] = z;
    dlx[HFIELD_HPP_N-1] = -Xc[HFIELD_HPP_N-1]+Xc[0];
    dly[HFIELD_HPP_N-1] = -Yc[HFIELD_HPP_N-1]+Yc[0];
}


/*
    For all the elements along coil, calculate dl cross R 
    dl cross R is the curl of vector dl and R
    XCross is X-component of the curl of dl and R, similarly I get Y and Z
*/
void HField::calculate_Cross_vector(){
    std::cout << "xCross,yCross,zCross,R \n";
    for (int i = 0; i < HFIELD_HPP_N; i++)
    {
        xCross[i] = dly[i]*Rz[i];
        yCross[i] = -dlx[i]*Rz[i];
        zCross[i] = (dlx[i]*Ry[i])-(dly[i]*Rx[i]);
        R[i] = std::sqrt(std::pow(Rx[i],2) + std::pow(Ry[i],2) + std::pow(Rz[i],2));
        std::cout << i << ": "<<   xCross[i] << "," << yCross[i] << "," << zCross[i] << "," << R[i] << "\n";
    }
}


/*
    This will be the biot savarts law equation
*/
void HField::calculate_BIOT_vector(){
    std::cout << "xB,yB,zB \n";
    for (int i = 0; i < HFIELD_HPP_N; i++)
    {
        xB[i] = I*mu0 / (4*PI*std::pow(R[i],3)) * xCross[i];
        yB[i] = I*mu0 / (4*PI*std::pow(R[i],3)) * yCross[i];
        zB[i] = I*mu0 / (4*PI*std::pow(R[i],3)) * zCross[i];
        std::cout << i << ": "<<   xB[i] << "," << yB[i] << "," << zB[i] << "\n";
    }
}

/*
    Method for calculating the H-field vector at a given position.
    transform the position so that it is relative to start position and avalanche transmitter
*/
V3D HField::getHFieldVector(double posLong, double posLat){
    //normalize position so that it is relative to lavinetransmitter and start position
    V3D posV;
    posV.x = posLong;
    posV.y = posLat;
    posV.z = 0;

    posV = calculate_Relative_Pos(posV);

    calculate_R_vector(posV.x, posV.y);
    calculate_Cross_vector();
    calculate_BIOT_vector();

    V3D returnVector;
    returnVector.x = 0;
    returnVector.y = 0;
    returnVector.z = 0;
    /*
        As the calculation for the coil is with the coil placees in the xy plane, we map the output different
        By mapping ycoil -> xcoordinate the coil is places along the x axises
        and mapping zcoil -> y coordinates then the H-field will point "forward" in the z-axis.
    */
    for (int i = 0; i < HFIELD_HPP_N; i++)
    {
        returnVector.x = returnVector.x + yB[i];
        returnVector.y = returnVector.y + zB[i];
        returnVector.z = returnVector.x + xB[i];
    }
    returnVector.print();

    return returnVector;
}

/*
Sets the lavine position relative to the startPos
Using x[meter] and y[meter] as offsets from start position
Resulting avalanche position is in lat,long
*/
void HField::setAvalanchePos(double x, double y){

    //currently in lat,long domain calculate offset in latitude and logitude in normal meters as that is input
    x = x * longConvertionFactor ;
    y = y * latConvertionFactor;
    std::cout <<"LC" << longConvertionFactor;
    std::cout <<"\nLC2" << latConvertionFactor;
    avalanchePos.x = startPos.x - x;
    avalanchePos.y = startPos.y - y;
    avalanchePos.z = 0;

    std::cout << "X: " <<  avalanchePos.x << " Y: " << avalanchePos.y << " Z: " << avalanchePos.z<< "\n";
}

/*
    This function is for setting the avalanche reciever as a offset from the start postion
    Done by just setting the venctor straight away
*/
void HField::setAvalanchePosFromOffset(double x, double y){
    avalanchePos.x = -x;
    avalanchePos.y = -y;
    avalanchePos.z = 0;
    std::cout << "X: " <<  avalanchePos.x << " Y: " << avalanchePos.y << " Z: " << avalanchePos.z<< "\n";
}

/*  Sets the drone start position which all calculation is normalized to
    x is the positive longitude means postive x will move start pos EAST
    y is the postive latitude which means positive y will move start pos north
*/
void HField::setStartPos(double x, double y){
    startPos.x = x;
    startPos.y = y;
    std::cout << "X: " <<  startPos.x << " Y: " << startPos.y<< "\n";
}

/*
    #MIKKEL ASCII GOD ART Theory
       c
   ------->
   \      ^
  a \    / b
     \  /
      v 

    Vector calculations
    c is the wanted vector
    a is the vector between avalanche transmitter and the start position
    b is the vector between start position and the given position
    Input vector should be in lat,long
    resulting vector is a meter vector
*/
V3D HField::calculate_Relative_Pos(V3D pos){
    V3D bV;
    //calculate b vector and convert vector from long,lat to meter
    bV.x = (pos.x-startPos.x)/latConvertionFactor;
    bV.y = (pos.y-startPos.y)/longConvertionFactor;
    bV.z = (pos.z-startPos.z);

    V3D cV;
    cV.x = (avalanchePos.x + bV.x);
    cV.y = (avalanchePos.x + bV.y);
    cV.z = 0;
    std::cout << "cV:  ";
    cV.print();
    return cV;
}


void HField::printStatus(){
    double Ra = 0.01; //radius of sending antenna
    double I = 0.1399; //current in coil
    int mu0 = 1; //u0
    std::cout << "Ra: " << 0.01 << "\n I: " << I << "\n mu0: " << mu0 << "\n";
    std::cout << "Startpos: " << startPos.x << "," << startPos.y << "," << startPos.z << "\n";
    std::cout << "Avalanche pos: " << avalanchePos.x << "," << avalanchePos.y << "," << avalanchePos.z << "\n";
}