#ifndef CMDWATCHDOG_H
#define CMDWATCHDOG_H

class CmdWatchDog{
public:
    CmdWatchDog(double time_out = 0.1){
        m_expire_duration.fromSec(time_out);
    }
    ~CmdWatchDog(){
        ros::shutdown();
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
#endif
