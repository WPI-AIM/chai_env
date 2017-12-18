#include "chai_env/Object.h"

Object::Object(std::string a_name): RosCom(a_name){

}

void Object::cur_position(double px, double py, double pz){
    m_trans.setOrigin(tf::Vector3(px, py, pz));
    m_poseStamped.pose.position.x = px;
    m_poseStamped.pose.position.y = py;
    m_poseStamped.pose.position.z = pz;
}

void Object::cur_orientation(double roll, double pitch, double yaw){
    tf::Quaternion rot_quat;
    rot_quat.setRPY(roll, pitch, yaw);
    m_trans.setRotation(rot_quat);
    tf::quaternionTFToMsg(rot_quat, m_poseStamped.pose.orientation);
}
