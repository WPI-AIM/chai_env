#include "chai_env/Object.h"
namespace chai_env{
Object::Object(std::string a_name): ObjectRosCom(a_name){

}

void Object::cur_position(double px, double py, double pz){
    m_trans.setOrigin(tf::Vector3(px, py, pz));
    m_objectState.pose_cur.position.x = px;
    m_objectState.pose_cur.position.y = py;
    m_objectState.pose_cur.position.z = pz;
}

void Object::cur_orientation(double roll, double pitch, double yaw){
    tf::Quaternion rot_quat;
    rot_quat.setRPY(roll, pitch, yaw);
    m_trans.setRotation(rot_quat);
    tf::quaternionTFToMsg(rot_quat, m_objectState.pose_cur.orientation);
}

void Object::cur_orientation(double qx, double qy, double qz, double qw){
    tf::Quaternion rot_quat(qx, qy, qz, qw);
    m_trans.setRotation(rot_quat);
    tf::quaternionTFToMsg(rot_quat, m_objectState.pose_cur.orientation);
}

void Object::cur_force(double fx, double fy, double fz){
    tf::Vector3 f(fx, fy, fz);
    tf::vector3TFToMsg(f, m_objectState.wrench_cur.force);
}

void Object::cur_torque(double nx, double ny, double nz){
    tf::Vector3 n(nx, ny, nz);
    tf::vector3TFToMsg(n, m_objectState.wrench_cur.torque);
}

void Object::set_chai_wall_time(double a_sec){
    m_objectState.chai_wall_time = a_sec;
    increment_sim_step();
    m_objectState.header.stamp = ros::Time::now();
}

void Object::set_chai_sim_time(double a_sec){
    m_objectState.chai_sim_time = a_sec;
}

void Object::set_mass(double a_mass){
    m_objectState.mass = a_mass;
}

void Object::set_principal_intertia(double Ixx, double Iyy, double Izz){
    m_objectState.pInertia.x = Ixx;
    m_objectState.pInertia.y = Iyy;
    m_objectState.pInertia.z = Izz;
}

void Object::increment_sim_step(){
    m_objectState.sim_step++;
}

void Object::set_sim_step(uint step){
    m_objectState.sim_step = step;
}


extern "C"{

Object* create_object(std::string name){
    return new Object(name);
}

void destroy_object(Object* obj){
    delete obj;
}

}

}
