#ifndef OBJECT_H
#define OBJECT_H

#include <string>
#include "chai_env/ObjectRosCom.h"

struct Cmd{
    Cmd(){
        Fx = Fy = Fz = 0;
        Nx = Ny = Nz = 0;
        J1 = J2 = J3 = 0;
    }
    void update(const chai_msgs::ObjectCmd* cmd){
        Fx = cmd->wrench.force.x;
        Fy = cmd->wrench.force.y;
        Fz = cmd->wrench.force.z;
        Nx = cmd->wrench.torque.x;
        Ny = cmd->wrench.torque.y;
        Nz = cmd->wrench.torque.z;
        J1 = cmd->joint_cmds[0];
        J2 = cmd->joint_cmds[1];
        J3 = cmd->joint_cmds[2];
    }

    double Fx, Fy, Fz;
    double Nx, Ny, Nz;
    double J1, J2, J3;
};

namespace chai_env{
class Object:public ObjectRosCom{
public:
    Object(std::string a_name);
    inline void set_name(std::string name){m_objectState.name.data = name;}
    void cur_position(double px, double py, double pz);
    void cur_orientation(double roll, double pitch, double yaw);
    void cur_orientation(double qx, double qy, double qz, double qw);
    void cur_force(double fx, double fy, double fz);
    void cur_torque(double nx, double ny, double nz);
    void update_cmd_from_ros();
    inline void set_chai_wall_time(double a_sec);
    inline void set_chai_sim_time(double a_sec);
    inline void set_mass(double a_mass);
    inline void set_principal_intertia(double Ixx, double Iyy, double Izz);
    inline void increment_sim_step();
    inline void set_sim_step(uint step);
    Cmd m_cmd;
};
}

#endif
