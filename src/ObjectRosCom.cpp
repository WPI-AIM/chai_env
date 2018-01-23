#include "chai_env/ObjectRosCom.h"

ObjectRosCom::ObjectRosCom(std::string a_name): RosComBase(a_name){
    init();
}

ObjectRosCom::ObjectRosCom(std::string a_name, int a_freq): RosComBase(a_name, a_freq){
    init();
}

void ObjectRosCom::init(){
    m_State.name.data = m_name;
    m_State.sim_step = 0;
    int argc = 0;
    char **argv = 0;
    ros::init(argc, argv, "chai_env_node");
    nodePtr.reset(new ros::NodeHandle);
    aspinPtr.reset(new ros::AsyncSpinner(5));
    ratePtr.reset(new ros::Rate(m_freq));
    nodePtr->setCallbackQueue(&m_custom_queue);

    m_chai_namespace = "chai/env";
    m_pub = nodePtr->advertise<chai_msgs::ObjectState>("/" + m_chai_namespace + "/" + m_name + "/State", 10);
    m_sub = nodePtr->subscribe("/" + m_chai_namespace + "/" + m_name + "/Command", 10, &ObjectRosCom::sub_cb, this);

    m_thread = boost::thread(boost::bind(&ObjectRosCom::run_publishers, this));
    std::cerr << "Thread Joined: " << m_name << std::endl;
    m_wd = CmdWatchDog(0.1);
}

ObjectRosCom::~ObjectRosCom(){
    ros::shutdown();
    std::cerr << "Thread ShutDown: " << m_State.name.data << std::endl;
}

void ObjectRosCom::reset_cmd(){
    m_Cmd.wrench.force.x = 0;
    m_Cmd.wrench.force.y = 0;
    m_Cmd.wrench.force.y = 0;
    m_Cmd.wrench.torque.x = 0;
    m_Cmd.wrench.torque.y = 0;
    m_Cmd.wrench.torque.y = 0;
    for(size_t idx = 0 ; idx < m_Cmd.joint_cmds.size() ; idx++){
        m_Cmd.joint_cmds[idx] = 0;
    }
}

void ObjectRosCom::sub_cb(chai_msgs::ObjectCmdConstPtr msg){
    m_Cmd.joint_cmds.resize(msg->joint_cmds.size());
    m_Cmd = *msg;
    m_wd.acknowledge_wd();
}

void ObjectRosCom::run_publishers(){
    while(nodePtr->ok()){
        m_pub.publish(m_State);
        m_custom_queue.callAvailable();
        if(m_wd.is_wd_expired()){
            m_wd.consolePrint(m_State.name.data);
            reset_cmd();
        }
        ratePtr->sleep();
    }
}
