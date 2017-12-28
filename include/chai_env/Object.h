#ifndef OBJECT_H
#define OBJECT_H

#include <string>
#include "chai_env/RosCom.h"

class Object: public RosCom{
public:
    Object(std::string a_name);
    inline void set_name(std::string name){m_objectState.name.data = name;}
    void cur_position(double px, double py, double pz);
    void cur_orientation(double roll, double pitch, double yaw);
    void cur_orientation(double qx, double qy, double qz, double qw);
    void cur_force(double fx, double fy, double fz);
    void cur_torque(double nx, double ny, double nz);
    void set_time_stamp(double n_sec);
    void set_mass(double a_mass);
    void set_principal_intertia(double Ixx, double Iyy, double Izz);
    void increment_sim_step();
    void set_sim_step(uint step);
    void set_wall_time(double wall_time);
};


#endif
