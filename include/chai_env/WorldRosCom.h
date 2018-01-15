#ifndef WORLDROSCOM_H
#define WORLDROSCOM_H

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <chai_msgs/WorldState.h>
#include <chai_msgs/WorldCmd.h>
#include <tf/tf.h>
#include <tf/LinearMath/Transform.h>
#include <ros/callback_queue.h>
#include <ros/duration.h>
#include "chai_env/CmdWatchDog.h"

class WorldRosCom{
public:
    WorldRosCom(std::string a_name, int a_freq = 1000);
    ~WorldRosCom();
    void run_publishers();

protected:
    boost::shared_ptr<ros::NodeHandle> nodePtr;
    boost::shared_ptr<ros::AsyncSpinner> aspinPtr;
    boost::shared_ptr<ros::Rate> ratePtr;
    int m_freq;
    CmdWatchDog m_wd;

    std::string chai_namespace;
    std::string m_name;
    ros::Publisher world_state_pub;
    ros::Subscriber world_state_sub;

    chai_msgs::WorldState m_worldState;
    chai_msgs::WorldCmd m_worldCmd;
    chai_msgs::WorldCmd m_worldCmdPrev;

    boost::thread m_thread;
    ros::CallbackQueue custom_queue;

    bool m_enableSimThrottle;
    bool m_stepSim;
    int m_num_skip_steps;
    int m_skip_steps_ctr;

    inline void reset_cmd();
    void world_sub_cb(chai_msgs::WorldCmdConstPtr msg);
};


#endif
