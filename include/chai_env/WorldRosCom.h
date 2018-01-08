#ifndef ROSCOM_H
#define ROSCOM_H

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <chai_msg/WorldState.h>
#include <chai_msg/WorldCmd.h>
#include <tf/tf.h>
#include <tf/LinearMath/Transform.h>
#include <ros/callback_queue.h>
#include <ros/duration.h>

class WorldRosCom{
    friend class World;
public:
    WorldRosCom(std::string a_name, int a_freq = 1000);
    ~WorldRosCom();
    void run_publishers();

private:
    boost::shared_ptr<ros::NodeHandle> nodePtr;
    boost::shared_ptr<ros::AsyncSpinner> aspinPtr;
    boost::shared_ptr<ros::Rate> ratePtr;
    int m_freq;

    std::string chai_namespace;
    std::string m_name;
    ros::Publisher world_state_pub;
    ros::Subscriber world_state_sub;

    chai_msg::WorldState m_worldState;
    chai_msg::WorldCmd m_worldCmd;

    boost::thread m_thread;
    ros::CallbackQueue custom_queue;

    void world_sub_cb(chai_msg::WorldCmdConstPtr msg);
};


#endif
