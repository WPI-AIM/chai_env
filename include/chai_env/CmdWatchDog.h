#ifndef CMDWATCHDOG_H
#define CMDWATCHDOG_H

class CmdWatchDog{
public:
    CmdWatchDog(double time_out = 0.1){
        m_expire_duration.fromSec(time_out);
        m_initialized = false;
    }
    void acknowledge_wd(){
        m_initialized = true;
        m_next_cmd_expected_time= ros::Time::now() + m_expire_duration;
    }
    bool is_wd_expired(){
        return (ros::Time::now() > m_next_cmd_expected_time && m_initialized) ? true : false;
    }
    void consolePrint(std::string class_name){
        if(m_initialized){
            std::cerr << "WatchDog expired, Resetting \"" << class_name << "\" command" << std::endl;
            m_initialized = false;
        }
    }

private:
    ros::Time m_next_cmd_expected_time;
    ros::Duration m_expire_duration;
    bool m_initialized;
};
#endif
