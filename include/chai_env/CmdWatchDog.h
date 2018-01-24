#ifndef CMDWATCHDOG_H
#define CMDWATCHDOG_H

#include <ros/rate.h>

class CmdWatchDog{
public:
    CmdWatchDog(const int &a_freq_min, const int&a_freq_max , double time_out = 0.5): m_freq_min(a_freq_min), m_freq_max(a_freq_max){
        m_expire_duration.fromSec(time_out);
        m_initialized = false;
        _m_minRatePtr.reset(new ros::Rate(m_freq_min));
        _m_maxRatePtr.reset(new ros::Rate(m_freq_max));
        m_ratePtr = _m_minRatePtr;
    }
    void acknowledge_wd(){
        if (m_initialized == false){ m_ratePtr = _m_maxRatePtr;}
        m_initialized = true;
        m_next_cmd_expected_time= ros::Time::now() + m_expire_duration;
    }
    bool is_wd_expired(){
        bool expired = (ros::Time::now() > m_next_cmd_expected_time && m_initialized) ? true : false;
        if(expired) m_ratePtr = _m_minRatePtr;
        return expired;
    }
    void consolePrint(std::string class_name){
        if(m_initialized){
            m_initialized = false;
            std::cerr << "WatchDog expired, Resetting \"" << class_name << "\" command" << std::endl;
        }
    }
    boost::shared_ptr<ros::Rate> m_ratePtr;

protected:
    int m_freq_min, m_freq_max;

private:
    boost::shared_ptr<ros::Rate> _m_minRatePtr, _m_maxRatePtr;
    ros::Time m_next_cmd_expected_time;
    ros::Duration m_expire_duration;
    bool m_initialized;
};
#endif
