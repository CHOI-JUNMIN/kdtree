#ifndef POINT3D_H
#define POINT3D_H

struct Point3D
{
    float x, y, z;
    Point3D(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}
};

#endif // POINT3D_H