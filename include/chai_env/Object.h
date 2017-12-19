#ifndef OBJECT_H
#define OBJECT_H

#include <string>
#include "chai_env/RosCom.h"

class Object: public RosCom{
public:
    Object(std::string a_name);
    inline void set_name(std::string name){m_name.data = name;}
    void cur_position(double px, double py, double pz);
    void cur_orientation(double roll, double pitch, double yaw);
    void cur_orientation(double qx, double qy, double qz, double qw);
    void cur_force(double fx, double fy, double fz);
    void cur_torque(double nx, double ny, double nz);
};


#endif
