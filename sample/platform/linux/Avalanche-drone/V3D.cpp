#include "V3D.hpp"
#include <iostream>

V3D::V3D(){
    x = 0;
    y = 0;
    z = 0;
}

V3D::~V3D(){
}

void V3D::print(){
    std::cout << x << "," << y << "," << z << "\n";
}