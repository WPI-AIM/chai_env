#ifndef OBJECTROSCOM_H
#define OBJECTROSCOM_H

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <geometry_msgs/WrenchStamped.h>
#include <chai_msg/ObjectState.h>
#include <tf/tf.h>
#include <tf/LinearMath/Transform.h>
#include <ros/callback_queue.h>
#include <ros/duration.h>

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

class CmdWatchDog{
public:
    CmdWatchDog(double time_out = 0.1){
        m_expire_duration.fromSec(time_out);
    }
    void acknowledge_wd(){
        m_next_cmd_expected_time= ros::Time::now() + m_expire_duration;
    }
    bool is_wd_expired(){
        return ros::Time::now() > m_next_cmd_expected_time ? true : false;
    }

private:
    ros::Time m_next_cmd_expected_time;
    ros::Duration m_expire_duration;
};

class ObjectRosCom: public CmdWatchDog{
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

    std::string chai_namespace;
    ros::Publisher obj_state_pub;
    ros::Subscriber obj_wrench_sub;

    tf::Transform m_trans;
    geometry_msgs::WrenchStamped m_wrenchStamped_Cmd;
    chai_msg::ObjectState m_objectState;

    boost::thread m_thread;
    ros::CallbackQueue custom_queue;

    void wrench_sub_cb(geometry_msgs::WrenchStampedConstPtr msg);
};


#endif
