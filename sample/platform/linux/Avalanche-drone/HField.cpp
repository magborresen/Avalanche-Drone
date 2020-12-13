#include "HField.hpp"
#include "SignalSearch.hpp"

HField::HField(/* args */)
{
    //initialize phi
    double i = -PI/2.0;
    while(i < 3*PI/2){
        phi.push_back(i);
        i +=2*PI/(HFIELD_HPP_N-1);
    }

    for (int i = 0; i < phi.size(); i++)
    {
        Xc.push_back( std::cos(phi[i]) * Ra);
    }
    for (int i = 0; i < phi.size(); i++)
    {
        Yc.push_back( std::sin(phi[i]) * Ra);
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
    for (int i = 0; i < (HFIELD_HPP_N-1); i++)
    {
        Rx[i] = -0.5 * (Xc[i] + Xc[i+1]);
        Ry[i] = (y - ( 0.5 * (Yc[i]+ Yc[i+1]) ));
        Rz[i] = z;
        dlx[i] = Xc[i+1]-Xc[i];
        dly[i] = Yc[i+1]-Xc[i];
    }
    //set the last element
    Rx[HFIELD_HPP_N] = -0.5 * (Xc[HFIELD_HPP_N-1] + Xc[0]);
    Ry[HFIELD_HPP_N] = (y - ( 0.5 * (Yc[HFIELD_HPP_N-1]+ Yc[0]) ));
    Rz[HFIELD_HPP_N] = z;
    dlx[HFIELD_HPP_N] = Xc[HFIELD_HPP_N-1]-Xc[0];
    dly[HFIELD_HPP_N] = Yc[HFIELD_HPP_N-1]-Xc[0];
}


/*
    For all the elements along coil, calculate dl cross R 
    dl cross R is the curl of vector dl and R
    XCross is X-component of the curl of dl and R, similarly I get Y and Z
*/
void HField::calculate_Cross_vector(){
    for (int i = 0; i < HFIELD_HPP_N; i++)
    {
        xCross[i] = dly[i]*Rz[i];
        yCross[i] = -dlx[i]*Rz[i];
        zCross[i] = (dlx[i]*Ry[i])-(dly[i]*Rx[i]);
        R[i] = std::sqrt(std::pow(Rx[i],2) + std::pow(Ry[i],2) + std::pow(Rz[i],2));
    }
}


/*
    This will be the biot savarts law equation
*/
void HField::calculate_BIOT_vector(){
    for (int i = 0; i < HFIELD_HPP_N; i++)
    {
        xB[i] = I*mu0 / (std::pow(R[i],3)) * xCross[i];
        yB[i] = I*mu0 / (std::pow(R[i],3)) * yCross[i];
        zB[i] = I*mu0 / (std::pow(R[i],3)) * zCross[i];
    }
}

/*
    Method for calculating the H-field vector at a given position.
    transform the position so that it is relative to start position and avalanche transmitter
*/
v3d HField::getHFieldVector(double posX, double posY){
    //normalize position so that it is relative to lavinetransmitter and start position
    v3d posV;
    posV.x = posX;
    posV.y = posY;
    posV.z = 0;

    posV = calculate_Relative_Pos(posV);

    calculate_R_vector(posV.x, posV.y);
    calculate_Cross_vector();
    calculate_BIOT_vector();

    v3d returnVector;
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
        returnVector.x = returnVector.x +yB[i];
        returnVector.y = returnVector.y +zB[i];
        returnVector.z = returnVector.x +xB[i];
    }
    return returnVector;
}

/*
Sets the lavine position relative to the startPos
Using x[meter] and y[meter] as offsets from start position
Resulting avalanche position is in lat,long
*/
void HField::setAvalanchePos(double x, double y){

    //currently in lat,long domain calculate offset in latitude and logitude in normal meters as that is input
    x = longConvertionFactor * x;
    y = latConvertionFactor * y;
    HField::avalanchePos.x = startPos.x - x;
    HField::avalanchePos.y = startPos.y - y;
    HField::avalanchePos.z = 0;
}

/*  Sets the drone start position wich all calculation is normalized to
    x is the positive longitude means postive x will move start pos EAST
    y is the postive latitude which means positive y will move start pos north
*/
void HField::setStartPos(double x, double y){
    HField::startPos.x = x;
    HField::startPos.y = y;
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
v3d HField::calculate_Relative_Pos(v3d pos){
    v3d bV;
    bV.x = pos.x-pos.x;
    bV.y = pos.y-pos.y;
    bV.z = pos.z-pos.z;

    v3d cV;
    cV.x = (avalanchePos.x + bV.x)*latConvertionFactor;
    cV.y = (avalanchePos.x + bV.y)*longConvertionFactor;
    cV.z = avalanchePos.x + bV.z;
    return cV;
}