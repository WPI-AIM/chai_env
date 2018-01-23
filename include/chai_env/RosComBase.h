#ifndef ROSCOMBASE_H
#define ROSCOMBASE_H

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <tf/tf.h>
#include <tf/LinearMath/Transform.h>
#include <ros/callback_queue.h>
#include <ros/duration.h>
#include "chai_env/CmdWatchDog.h"

template <class T_state, class T_cmd>
class RosComBase{
public:
    RosComBase(std::string a_name, int a_freq = 5000){m_name = a_name; m_freq = a_freq;}
    virtual void init() = 0;
    virtual void run_publishers();

protected:
    boost::shared_ptr<ros::NodeHandle> nodePtr;
    boost::shared_ptr<ros::AsyncSpinner> aspinPtr;
    boost::shared_ptr<ros::Rate> ratePtr;
    int m_freq;
    CmdWatchDog m_wd;

    std::string m_chai_namespace;
    std::string m_name;
    ros::Publisher m_pub;
    ros::Subscriber m_sub;

    tf::Transform m_trans;
    T_state m_State;
    T_cmd m_Cmd;
    T_cmd m_CmdPrev;

    boost::thread m_thread;
    ros::CallbackQueue m_custom_queue;

    virtual void reset_cmd() = 0;
};

template<class T_state, class T_cmd>
void RosComBase<T_state, T_cmd>::run_publishers(){
    while(nodePtr->ok()){
        m_pub.publish(m_State);
        m_custom_queue.callAvailable();
        if(m_wd.is_wd_expired()){
            m_wd.consolePrint(m_name);
            reset_cmd();
        }
        ratePtr->sleep();
    }
}


#endif
