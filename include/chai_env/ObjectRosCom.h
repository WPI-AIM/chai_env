#ifndef OBJECTROSCOM_H
#define OBJECTROSCOM_H

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <geometry_msgs/WrenchStamped.h>
#include <chai_msgs/ObjectState.h>
#include <tf/tf.h>
#include <tf/LinearMath/Transform.h>
#include <ros/callback_queue.h>
#include <ros/duration.h>
#include "chai_env/CmdWatchDog.h"

struct WrenchCmd{
    WrenchCmd(){
        Fx = Fy = Fz = 0;
        Nx = Ny = Nz = 0;
    }
    void clear_wrench(){
        Fx = Fy = Fz = 0;
        Nx = Ny = Nz = 0;
    }

    double Fx, Fy, Fz;
    double Nx, Ny, Nz;
};

class ObjectRosCom{
public:
    ObjectRosCom(std::string a_name, int a_freq = 1000);
    ~ObjectRosCom();
    void run_publishers();
    WrenchCmd m_wrenchCmd;


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
    geometry_msgs::WrenchStamped m_wrenchStamped_Cmd;
    chai_msgs::ObjectState m_objectState;

    boost::thread m_thread;
    ros::CallbackQueue custom_queue;

    void wrench_sub_cb(geometry_msgs::WrenchStampedConstPtr msg);
};


#endif
