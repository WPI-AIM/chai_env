#include "chai_env/chai_env.h"

ChaiEnv::ChaiEnv(){
    m_numObjects = 0;
}


void ChaiEnv::create_object(std::string name){
    m_objectMap[name] = boost::shared_ptr<Object>(new Object(name));
}

bool ChaiEnv::set_object_position(std::string name, double px, double py, double pz){
    m_objectIt = m_objectMap.find(name);
    if(m_objectIt != m_objectMap.end()){
        m_objectMap[name]->set_position(px, py, pz);
        return true;
    }
    else{
        return false;
    }
}

bool ChaiEnv::set_object_orientation(std::string name, double roll, double pitch, double yaw){
    m_objectIt = m_objectMap.find(name);
    if(m_objectIt != m_objectMap.end()){
        m_objectMap[name]->set_orientation(roll, pitch, yaw);
        return true;
    }
    else{
        return false;
    }
}

ChaiEnv::~ChaiEnv(){

}
