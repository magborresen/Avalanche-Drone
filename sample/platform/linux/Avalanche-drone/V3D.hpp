#ifndef V3D_HPP
#define V3D_HPP

class V3D
{
private:
    /* data */
public:
    double x;
    double y;
    double z;
    V3D();
    V3D(double ix,double iy,double iz);
    ~V3D();
};

V3D::V3D(){
    x = 0;
    y = 0;
    z = 0;
}

V3D::~V3D(){
}

#endif