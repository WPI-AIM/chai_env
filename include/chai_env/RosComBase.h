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
    RosComBase(std::string a_name, int a_freq_min = 10, int a_freq_max = 2500)
    {
        m_name = a_name;
        m_chai_namespace = "chai/env";

        int argc = 0;
        char **argv = 0;
        ros::init(argc, argv, "chai_env_node");
        nodePtr.reset(new ros::NodeHandle);
        aspinPtr.reset(new ros::AsyncSpinner(5));
        nodePtr->setCallbackQueue(&m_custom_queue);
        m_watchDogPtr.reset(new CmdWatchDog(a_freq_min, a_freq_max));
    }
    virtual void init() = 0;
    virtual void run_publishers();

protected:
    boost::shared_ptr<ros::NodeHandle> nodePtr;
    boost::shared_ptr<ros::AsyncSpinner> aspinPtr;
    boost::shared_ptr<CmdWatchDog> m_watchDogPtr;

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
        if(m_watchDogPtr->is_wd_expired()){
            m_watchDogPtr->consolePrint(m_name);
            reset_cmd();
        }
        m_watchDogPtr->m_ratePtr->sleep();
    }
}


#endif
