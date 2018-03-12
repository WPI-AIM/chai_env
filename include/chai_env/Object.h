#ifndef OBJECT_H
#define OBJECT_H

#include <string>
#include "chai_env/ObjectRosCom.h"

struct AFCmd{
    AFCmd(){
        Fx = Fy = Fz = 0;
        Nx = Ny = Nz = 0;
        size_J_cmd = 0;
    }
    void update(const chai_msgs::ObjectCmd* cmd){
        px = cmd->pose.position.x;
        py = cmd->pose.position.y;
        pz = cmd->pose.position.z;
        qx = cmd->pose.orientation.x;
        qy = cmd->pose.orientation.y;
        qz = cmd->pose.orientation.z;
        qw = cmd->pose.orientation.w;

        Fx = cmd->wrench.force.x;
        Fy = cmd->wrench.force.y;
        Fz = cmd->wrench.force.z;
        Nx = cmd->wrench.torque.x;
        Ny = cmd->wrench.torque.y;
        Nz = cmd->wrench.torque.z;
        size_J_cmd = cmd->joint_cmds.size();
        J_cmd.resize(size_J_cmd);
        pos_ctrl = cmd->pos_ctrl;
        for(size_t idx = 0; idx < size_J_cmd ; idx++){
            J_cmd[idx] = cmd->joint_cmds[idx];
        }
    }
    double px, py, pz;
    double qx, qy, qz, qw;
    double Fx, Fy, Fz;
    double Nx, Ny, Nz;
    std::vector<double> J_cmd;
    size_t size_J_cmd;
    // By default, always consider force control, unless pos_ctrl is set to true
    bool pos_ctrl;
};

namespace chai_env{
class Object:public ObjectRosCom{
public:
    Object(std::string a_name);
    inline void set_name(std::string name){m_State.name.data = name;}
    void cur_position(double px, double py, double pz);
    void cur_orientation(double roll, double pitch, double yaw);
    void cur_orientation(double qx, double qy, double qz, double qw);
    void cur_force(double fx, double fy, double fz);
    void cur_torque(double nx, double ny, double nz);
    void update_af_cmd();
    void set_chai_wall_time(double a_sec);
    inline void set_chai_sim_time(double a_sec){ m_State.chai_sim_time = a_sec;}
    inline void set_mass(double a_mass){m_State.mass = a_mass;}
    inline void set_principal_intertia(double Ixx, double Iyy, double Izz){m_State.pInertia.x = Ixx; m_State.pInertia.y = Iyy; m_State.pInertia.z = Izz;}
    inline void increment_sim_step(){m_State.sim_step++;}
    inline void set_sim_step(uint step){m_State.sim_step = step;}
    AFCmd m_afCmd;
};
}

#endif
