#ifndef HFIELD_HPP
#define HFIELD_HPP

#include <vector>
#include <cmath>


#define PI 3.14159265359

//sets the resolution of coil circle
#define HFIELD_HPP_N 25

class HField
{
private:
    
    double Ra = 0.01; //radius of sending antenna
    double I = 0.1399; //current in coil
    int mu0 = 1; //u0
    std::vector<double> phi;
    std::vector<double> Xc; //X coordinates of the coil
    std::vector<double> Yc; //Y coordinates of the coil

    //arrays used for calculating the H-field
    double Rx[HFIELD_HPP_N];
    double Ry[HFIELD_HPP_N];
    double Rz[HFIELD_HPP_N];
    double dlx[HFIELD_HPP_N];
    double dly[HFIELD_HPP_N];
    double xCross[HFIELD_HPP_N];
    double yCross[HFIELD_HPP_N];
    double zCross[HFIELD_HPP_N];
    double R[HFIELD_HPP_N];
    double xB[HFIELD_HPP_N];
    double yB[HFIELD_HPP_N];
    double zB[HFIELD_HPP_N];

    //Method for calculating the H-field
    void calculate_R_vector(double y , double z);
    void calculate_Cross_vector();
    void calculate_BIOT_vector();

    //Method for calculating postion relative to normalize avalanche transmitter
    V3D calculate_Relative_Pos(V3D pos);

    //artribute for start position of drone
    V3D startPos;

    //artribute for avalanche transmitter position of drone relative to start position
    V3D avalanchePos;

    
public:
    
    void setAvalanchePos(double x, double y);
    void setStartPos(double x, double y);

    HField(/* args */);
    V3D getHFieldVector(double posX, double posY);
    ~HField();
};


#endif