#include "chai_env/ObjectRosCom.h"

ObjectRosCom::ObjectRosCom(std::string a_name): RosComBase(a_name){
    init();
}

ObjectRosCom::ObjectRosCom(std::string a_name, int a_freq_min, int a_freq_max): RosComBase(a_name, a_freq_min, a_freq_max){
    init();
}

void ObjectRosCom::init(){
    m_State.name.data = m_name;
    m_State.sim_step = 0;

    m_pub = nodePtr->advertise<chai_msgs::ObjectState>("/" + m_chai_namespace + "/" + m_name + "/State", 10);
    m_sub = nodePtr->subscribe("/" + m_chai_namespace + "/" + m_name + "/Command", 10, &ObjectRosCom::sub_cb, this);

    m_thread = boost::thread(boost::bind(&ObjectRosCom::run_publishers, this));
    std::cerr << "Thread Joined: " << m_name << std::endl;
}

ObjectRosCom::~ObjectRosCom(){
    ros::shutdown();
    std::cerr << "Thread ShutDown: " << m_State.name.data << std::endl;
}

void ObjectRosCom::reset_cmd(){
    m_Cmd.pos_ctrl = false;
    m_Cmd.wrench.force.x = 0;
    m_Cmd.wrench.force.y = 0;
    m_Cmd.wrench.force.z = 0;
    m_Cmd.wrench.torque.x = 0;
    m_Cmd.wrench.torque.y = 0;
    m_Cmd.wrench.torque.z = 0;
    for(size_t idx = 0 ; idx < m_Cmd.joint_cmds.size() ; idx++){
        m_Cmd.joint_cmds[idx] = 0;
    }
}

void ObjectRosCom::sub_cb(chai_msgs::ObjectCmdConstPtr msg){
    if (m_Cmd.joint_cmds.size() != msg->joint_cmds.size()){
      m_Cmd.joint_cmds.resize(msg->joint_cmds.size());
    }
    m_Cmd = *msg;
    m_watchDogPtr->acknowledge_wd();
}

