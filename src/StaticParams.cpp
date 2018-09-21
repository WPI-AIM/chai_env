#include "chai_env/StaticParams.h"

boost::shared_ptr<ros::NodeHandle> StaticParams::nodePtr;
std::string StaticParams::m_chai_namespace;
int StaticParams::m_usage_ctr;

StaticParams::StaticParams(){
    std::cerr << "Usage Counter is " << m_usage_ctr << std::endl;
    if (m_usage_ctr == 0){
        std::cerr << "Initing this many times " << m_usage_ctr << std::endl;
        m_chai_namespace = "chai/env";
        int argc = 0;
        char **argv = 0;
        ros::init(argc, argv, "chai_env_node");
        nodePtr.reset(new ros::NodeHandle);
    }
    m_usage_ctr ++;
}
