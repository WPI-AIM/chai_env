#ifndef ROSCOM_H
#define ROSCOM_H

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf/LinearMath/Transform.h>

class RosCom{
    friend class Object;
    RosCom(std::string a_name, int a_freq = 1000);
    ~RosCom();
    void run_publishers();

private:
    boost::shared_ptr<ros::NodeHandle> nodePtr;
    boost::shared_ptr<ros::AsyncSpinner> aspinPtr;
    boost::shared_ptr<ros::Rate> ratePtr;
    int m_freq;

    std::string chai_namespace;
    ros::Publisher obj_pose_pub;
    ros::Publisher obj_name_pub;
    ros::Publisher obj_wrench_pub;
    ros::Publisher obj_reward_pub;

    std_msgs::String m_name;
    tf::Transform m_trans;
    geometry_msgs::PoseStamped m_poseStamped;
    geometry_msgs::WrenchStamped m_wrenchStamped;
    std_msgs::Float32 m_reward;

    boost::thread m_thread;
};


#endif
