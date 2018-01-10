#include "chai_env/WorldRosCom.h"

WorldRosCom::WorldRosCom( std::string a_name, int a_freq){
    m_freq = a_freq;
    m_name = a_name;
    m_worldState.sim_step = 0;
    m_pauseSim = false;
    m_enableSimThrottle = false;
    int argc = 0;
    char **argv = 0;
    ros::init(argc, argv, "chai_env_node");
    nodePtr.reset(new ros::NodeHandle);
    aspinPtr.reset(new ros::AsyncSpinner(1));
    ratePtr.reset(new ros::Rate(m_freq));
    nodePtr->setCallbackQueue(&custom_queue);
    m_wd = CmdWatchDog(0.5);

    chai_namespace = "chai/env";
    world_state_pub = nodePtr->advertise<chai_msg::WorldState>("/" + chai_namespace + "/" + a_name + "/State", 10);
    world_state_sub = nodePtr->subscribe("/" + chai_namespace + "/" + a_name + "/Command", 10, &WorldRosCom::world_sub_cb, this);

    m_thread = boost::thread(boost::bind(&WorldRosCom::run_publishers, this));
    std::cerr << "Thread Joined: " << a_name << std::endl;
}

WorldRosCom::~WorldRosCom(){
    ros::shutdown();
    std::cerr << "Shutting Down: " << m_name << std::endl;
}

void WorldRosCom::world_sub_cb(chai_msg::WorldCmdConstPtr msg){
    m_worldCmdPrev = m_worldCmd;
    m_worldCmd = *msg;
    m_enableSimThrottle = (bool)m_worldCmd.enable_step_throttling;
    if (m_enableSimThrottle){
        if(m_pauseSim){
            m_pauseSim = !((bool)m_worldCmd.step ^ (bool)m_worldCmdPrev.step);
        }
    }
    else{
            m_pauseSim = false;
    }
    m_wd.acknowledge_wd();
}

void WorldRosCom::run_publishers(){
    while(nodePtr->ok()){
        world_state_pub.publish(m_worldState);
        custom_queue.callAvailable();
        if(m_wd.is_wd_expired()){
           m_enableSimThrottle = false;
           m_pauseSim = false;
        }
        ratePtr->sleep();
    }
}