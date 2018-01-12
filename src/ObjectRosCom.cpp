#include "chai_env/ObjectRosCom.h"

ObjectRosCom::ObjectRosCom( std::string a_name, int a_freq){
    m_freq = a_freq;
    m_objectState.name.data = a_name;
    m_objectState.sim_step = 0;
    int argc = 0;
    char **argv = 0;
    ros::init(argc, argv, "chai_env_node");
    nodePtr.reset(new ros::NodeHandle);
    aspinPtr.reset(new ros::AsyncSpinner(5));
    ratePtr.reset(new ros::Rate(m_freq));
    nodePtr->setCallbackQueue(&custom_queue);

    chai_namespace = "chai/env";
    obj_state_pub = nodePtr->advertise<chai_msgs::ObjectState>("/" + chai_namespace + "/" + a_name + "/State", 10);
    obj_wrench_sub = nodePtr->subscribe("/" + chai_namespace + "/" + a_name + "/Command", 10, &ObjectRosCom::cmd_sub_cb, this);

    m_thread = boost::thread(boost::bind(&ObjectRosCom::run_publishers, this));
    std::cerr << "Thread Joined: " << a_name << std::endl;
    m_wd = CmdWatchDog(0.1);
}

ObjectRosCom::~ObjectRosCom(){
    ros::shutdown();
    std::cerr << "Thread ShutDown: " << m_objectState.name.data << std::endl;
}

void ObjectRosCom::reset_cmd(){
    m_objectCmd.wrench.force.x = 0;
    m_objectCmd.wrench.force.y = 0;
    m_objectCmd.wrench.force.y = 0;
    m_objectCmd.wrench.torque.x = 0;
    m_objectCmd.wrench.torque.y = 0;
    m_objectCmd.wrench.torque.y = 0;
    m_objectCmd.joint_cmds[0] = 0;
    m_objectCmd.joint_cmds[1] = 0;
    m_objectCmd.joint_cmds[2] = 0;
}

void ObjectRosCom::cmd_sub_cb(chai_msgs::ObjectCmdConstPtr msg){
    m_objectCmd = *msg;
    m_wd.acknowledge_wd();
}

void ObjectRosCom::run_publishers(){
    while(nodePtr->ok()){
        obj_state_pub.publish(m_objectState);
        custom_queue.callAvailable();
        if(m_wd.is_wd_expired()){
            m_wd.consolePrint(m_objectState.name.data);
            reset_cmd();
        }
        ratePtr->sleep();
    }
}
