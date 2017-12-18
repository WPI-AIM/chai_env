#include "chai_env/RosCom.h"

RosCom::RosCom( std::string a_name, int a_freq){
    m_freq = a_freq;
    m_name.data = a_name;
    int argc = 0;
    char **argv = 0;
    ros::init(argc, argv, "chai_env_node");
    nodePtr.reset(new ros::NodeHandle);
    aspinPtr.reset(new ros::AsyncSpinner(5));
    ratePtr.reset(new ros::Rate(m_freq));


    chai_namespace = "chai/env";
    obj_name_pub = nodePtr->advertise<std_msgs::String>("/" + chai_namespace + "/" + m_name.data + "/Name", 10);
    obj_pose_pub = nodePtr->advertise<geometry_msgs::PoseStamped>("/" + chai_namespace + "/" + m_name.data + "/PoseStamped", 10);
    obj_wrench_pub = nodePtr->advertise<geometry_msgs::WrenchStamped>("/" + chai_namespace + "/" + m_name.data + "/WrenchStamped", 10);
    obj_reward_pub = nodePtr->advertise<std_msgs::Float32>("/" + chai_namespace + "/" + m_name.data + "/Reward", 10);

    m_thread = boost::thread(boost::bind(&RosCom::run_publishers, this));
    std::cerr << "Thread Joined: " << m_name.data << std::endl;
}

RosCom::~RosCom(){
    ros::shutdown();
    std::cerr << "Shutting Down: " << m_name.data << std::endl;
}

void RosCom::run_publishers(){

    while(nodePtr->ok()){
        obj_name_pub.publish(m_name);
        obj_pose_pub.publish(m_poseStamped);
        obj_wrench_pub.publish(m_wrenchStamped);
        obj_reward_pub.publish(m_reward);
        ratePtr->sleep();
    }
}
