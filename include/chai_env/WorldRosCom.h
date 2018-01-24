#ifndef WORLDROSCOM_H
#define WORLDROSCOM_H

#include "chai_env/RosComBase.h"
#include <chai_msgs/WorldState.h>
#include <chai_msgs/WorldCmd.h>

class WorldRosCom: public RosComBase<chai_msgs::WorldState, chai_msgs::WorldCmd>{
public:
    WorldRosCom(std::string a_name);
    WorldRosCom(std::string a_name, int a_freq_min, int a_freq_max);
    ~WorldRosCom();
    virtual void init();

protected:
    bool m_enableSimThrottle;
    bool m_stepSim;
    int m_num_skip_steps;
    int m_skip_steps_ctr;
    virtual void reset_cmd();
    void sub_cb(chai_msgs::WorldCmdConstPtr msg);
};


#endif
