#include "chai_env/WorldRosCom.h"

WorldRosCom::WorldRosCom(std::string a_name): RosComBase(a_name){
    init();
}

WorldRosCom::WorldRosCom(std::string a_name, int a_freq_min, int a_freq_max): RosComBase(a_name, a_freq_min, a_freq_max){
    init();
}

void WorldRosCom::init(){
    m_State.sim_step = 0;
    m_enableSimThrottle = false;
    m_stepSim = true;

    m_pub = nodePtr->advertise<chai_msgs::WorldState>("/" + m_chai_namespace + "/" + m_name + "/State", 10);
    m_sub = nodePtr->subscribe("/" + m_chai_namespace + "/" + m_name + "/Command", 10, &WorldRosCom::sub_cb, this);

    m_thread = boost::thread(boost::bind(&WorldRosCom::run_publishers, this));
    std::cerr << "Thread Joined: " << m_name << std::endl;
}

WorldRosCom::~WorldRosCom(){
    ros::shutdown();
    std::cerr << "Thread Shutdown: " << m_name << std::endl;
}

void WorldRosCom::reset_cmd(){
    m_enableSimThrottle = false;
    m_stepSim = true;
}

void WorldRosCom::sub_cb(chai_msgs::WorldCmdConstPtr msg){
    m_CmdPrev = m_Cmd;
    m_Cmd = *msg;
    m_num_skip_steps = m_Cmd.n_skip_steps;
    m_enableSimThrottle = (bool)m_Cmd.enable_step_throttling;
    if (m_enableSimThrottle){
        if(!m_stepSim){
            m_stepSim = (bool)m_Cmd.step_clock ^ (bool)m_CmdPrev.step_clock;
        }
    }
    else{
            m_stepSim = true;
    }
    m_watchDogPtr->acknowledge_wd();
}
