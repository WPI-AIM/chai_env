#include "chai_env/WorldRosCom.h"

WorldRosCom::WorldRosCom( std::string a_name, int a_freq){
    m_freq = a_freq;
    m_name = a_name;
    m_worldState.sim_step = 0;
    m_enableSimThrottle = false;
    m_stepSim = true;
    int argc = 0;
    char **argv = 0;
    ros::init(argc, argv, "chai_env_node");
    nodePtr.reset(new ros::NodeHandle);
    aspinPtr.reset(new ros::AsyncSpinner(1));
    ratePtr.reset(new ros::Rate(m_freq));
    nodePtr->setCallbackQueue(&custom_queue);

    chai_namespace = "chai/env";
    world_state_pub = nodePtr->advertise<chai_msgs::WorldState>("/" + chai_namespace + "/" + a_name + "/State", 10);
    world_state_sub = nodePtr->subscribe("/" + chai_namespace + "/" + a_name + "/Command", 10, &WorldRosCom::world_sub_cb, this);

    m_thread = boost::thread(boost::bind(&WorldRosCom::run_publishers, this));
    std::cerr << "Thread Joined: " << a_name << std::endl;
    m_wd = CmdWatchDog(0.5);
}

WorldRosCom::~WorldRosCom(){
    ros::shutdown();
    std::cerr << "Thread Shutdown: " << m_name << std::endl;
}

void WorldRosCom::reset_cmd(){
    m_enableSimThrottle = false;
    m_stepSim = true;
}

void WorldRosCom::world_sub_cb(chai_msgs::WorldCmdConstPtr msg){
    m_worldCmdPrev = m_worldCmd;
    m_worldCmd = *msg;
    m_enableSimThrottle = (bool)m_worldCmd.enable_step_throttling;
    if (m_enableSimThrottle){
        if(!m_stepSim){
            m_stepSim = (bool)m_worldCmd.step_clock ^ (bool)m_worldCmdPrev.step_clock;
        }
    }
    else{
            m_stepSim = true;
    }
    m_wd.acknowledge_wd();
}

void WorldRosCom::run_publishers(){
    while(nodePtr->ok()){
        world_state_pub.publish(m_worldState);
        custom_queue.callAvailable();
        if(m_wd.is_wd_expired()){
            reset_cmd();
        }
        ratePtr->sleep();
    }
}
