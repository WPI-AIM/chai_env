#ifndef CHAI_ENV_H
#define CHAI_ENV_H
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <tf/LinearMath/Transform.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>




struct Observation{
public:
    Observation();

    geometry_msgs::PoseStamped m_nextState;
    double m_reward;
    double m_done;
};

class Object{
public:
    Object();
    inline void set_name(std::string name){m_name.data = name;}
    void set_position(double px, double py, double pz);
    void set_orientation(double roll, double pitch, double yaw);
    std_msgs::String m_name;
    tf::Transform m_trans;
    geometry_msgs::PoseStamped m_poseStamped;
    geometry_msgs::WrenchStamped m_wrenchStamped;
    std_msgs::Float32 m_reward;
};

void Object::set_position(double px, double py, double pz){
    m_trans.setOrigin(tf::Vector3(px, py, pz));
    m_poseStamped.pose.position.x = px;
    m_poseStamped.pose.position.y = py;
    m_poseStamped.pose.position.z = pz;
}

void Object::set_orientation(double roll, double pitch, double yaw){
    tf::Quaternion rot_quat;
    rot_quat.setRPY(roll, pitch, yaw);
    m_trans.setRotation(rot_quat);
    tf::quaternionTFToMsg(rot_quat, m_poseStamped.pose.orientation);
}

class RosCom: public Object{
    RosCom();
    void run_publishers();

private:
    boost::shared_ptr<ros::NodeHandle> nodePtr;
    boost::shared_ptr<ros::AsyncSpinner> aspinPtr;

    std::string chai_namespace;
    ros::Publisher obj_pose_pub;
    ros::Publisher obj_name_pub;
    ros::Publisher obj_wrench_pub;
    ros::Publisher obj_reward_pub;
};

RosCom::RosCom(){
    int argc = 0;
    char **argv = 0;
    ros::init(argc, argv, "chai_env_node");
    nodePtr.reset(new ros::NodeHandle);
    aspinPtr.reset(new ros::AsyncSpinner(5));


    chai_namespace = "chai/env";
    obj_name_pub = nodePtr->advertise<std_msgs::String>("/" + chai_namespace + "/Name", 10);
    obj_pose_pub = nodePtr->advertise<geometry_msgs::PoseStamped>("/" + chai_namespace + "/PoseStamped", 10);
    obj_wrench_pub = nodePtr->advertise<geometry_msgs::WrenchStamped>("/" + chai_namespace + "/WrenchStamped", 10);
    obj_reward_pub = nodePtr->advertise<std_msgs::Float32>("/" + chai_namespace + "/Reward", 10);

}

void RosCom::run_publishers(){
    obj_name_pub.publish(m_name);
    obj_pose_pub.publish(m_poseStamped);
    obj_wrench_pub.publish(m_wrenchStamped);
    obj_reward_pub.publish(m_reward);
}

class ChaiEnv{
public:
    ChaiEnv();
    ~ChaiEnv();
    void create_object(std::string name);
    bool set_object_position(std::string name, double px, double py, double pz);
    bool set_object_orientation(std::string name, double roll, double pitch, double yaw);


private:
    static const int max_obj_size=10;
    boost::shared_ptr<ros::AsyncSpinner> spinnerPtr;
    boost::shared_ptr<ros::NodeHandle> nodePtr;
    int m_numObjects;
    std::map<std::string, boost::shared_ptr<Object> > m_objectMap;
    std::map<std::string, boost::shared_ptr<Object> >::iterator m_objectIt;
    boost::shared_ptr<Object> m_Objects[max_obj_size];
};


#endif
