#include "chai_env/WorldRosCom.h"

WorldRosCom::WorldRosCom( std::string a_name, int a_freq){
    m_freq = a_freq;
    m_name = a_name;
    m_worldState.sim_step = 0;
    int argc = 0;
    char **argv = 0;
    ros::init(argc, argv, "chai_env_node");
    nodePtr.reset(new ros::NodeHandle);
    aspinPtr.reset(new ros::AsyncSpinner(1));
    ratePtr.reset(new ros::Rate(m_freq));
    nodePtr->setCallbackQueue(&custom_queue);


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
    m_worldCmd = *msg;
}

void WorldRosCom::run_publishers(){
    while(nodePtr->ok()){
        world_state_pub.publish(m_worldState);
        custom_queue.callAvailable();
        ratePtr->sleep();
    }
}
