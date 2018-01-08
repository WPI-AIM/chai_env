#ifndef OBJECT_H
#define OBJECT_H

#include <string>
#include "chai_env/ObjectRosCom.h"

class Object: public ObjectRosCom{
public:
    Object(std::string a_name);
    inline void set_name(std::string name){m_objectState.name.data = name;}
    void cur_position(double px, double py, double pz);
    void cur_orientation(double roll, double pitch, double yaw);
    void cur_orientation(double qx, double qy, double qz, double qw);
    void cur_force(double fx, double fy, double fz);
    void cur_torque(double nx, double ny, double nz);
    void set_chai_wall_time(double a_sec);
    void set_chai_sim_time(double a_sec);
    void set_mass(double a_mass);
    void set_principal_intertia(double Ixx, double Iyy, double Izz);
    void increment_sim_step();
    void set_sim_step(uint step);
};


#endif
