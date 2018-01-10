#ifndef OBJECTROSCOM_H
#define OBJECTROSCOM_H

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <geometry_msgs/WrenchStamped.h>
#include <chai_msgs/ObjectState.h>
#include <chai_msgs/ObjectCmd.h>
#include <tf/tf.h>
#include <tf/LinearMath/Transform.h>
#include <ros/callback_queue.h>
#include <ros/duration.h>
#include "chai_env/CmdWatchDog.h"

class ObjectRosCom{
public:
    ObjectRosCom(std::string a_name, int a_freq = 1000);
    ~ObjectRosCom();
    void run_publishers();


protected:
    boost::shared_ptr<ros::NodeHandle> nodePtr;
    boost::shared_ptr<ros::AsyncSpinner> aspinPtr;
    boost::shared_ptr<ros::Rate> ratePtr;
    int m_freq;
    CmdWatchDog m_wd;

    std::string chai_namespace;
    ros::Publisher obj_state_pub;
    ros::Subscriber obj_wrench_sub;

    tf::Transform m_trans;
    chai_msgs::ObjectCmd m_objectCmd;
    chai_msgs::ObjectState m_objectState;

    boost::thread m_thread;
    ros::CallbackQueue custom_queue;

    void reset_cmd();
    void cmd_sub_cb(chai_msgs::ObjectCmdConstPtr msg);
};


#endif
