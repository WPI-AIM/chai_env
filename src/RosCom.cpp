#include "chai_env/RosCom.h"

RosCom::RosCom( std::string a_name, int a_freq){
    m_freq = a_freq;
    m_objectState.name.data = a_name;
    int argc = 0;
    char **argv = 0;
    ros::init(argc, argv, "chai_env_node");
    nodePtr.reset(new ros::NodeHandle);
    aspinPtr.reset(new ros::AsyncSpinner(5));
    ratePtr.reset(new ros::Rate(m_freq));
    nodePtr->setCallbackQueue(&custom_queue);


    chai_namespace = "chai/env";
    obj_state_pub = nodePtr->advertise<chai_msg::ObjectState>("/" + chai_namespace + "/" + a_name + "/State", 10);
    obj_wrench_sub = nodePtr->subscribe("/" + chai_namespace + "/" + a_name + "/Command", 10, &RosCom::wrench_sub_cb, this);

    m_thread = boost::thread(boost::bind(&RosCom::run_publishers, this));
    std::cerr << "Thread Joined: " << a_name << std::endl;
}

RosCom::~RosCom(){
    ros::shutdown();
    std::cerr << "Shutting Down: " << m_objectState.name.data << std::endl;
}

void RosCom::wrench_sub_cb(geometry_msgs::WrenchStampedConstPtr msg){
    m_wrenchStamped_Cmd = *msg;
    m_wrenchCmd.Fx = m_wrenchStamped_Cmd.wrench.force.x;
    m_wrenchCmd.Fy = m_wrenchStamped_Cmd.wrench.force.y;
    m_wrenchCmd.Fz = m_wrenchStamped_Cmd.wrench.force.z;
    m_wrenchCmd.Nx = m_wrenchStamped_Cmd.wrench.torque.x;
    m_wrenchCmd.Ny = m_wrenchStamped_Cmd.wrench.torque.y;
    m_wrenchCmd.Nz = m_wrenchStamped_Cmd.wrench.torque.z;
}

void RosCom::run_publishers(){

    while(nodePtr->ok()){
        obj_state_pub.publish(m_objectState);
        custom_queue.callAvailable();
        ratePtr->sleep();
    }
}
