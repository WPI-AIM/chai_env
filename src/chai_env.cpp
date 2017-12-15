#include "chai_env/chai_env.h"

ChaiEnv::ChaiEnv(){
    m_numObjects = 0;
}


void ChaiEnv::create_object(std::string name){
    m_objectMap[name] = boost::shared_ptr<Object>(new Object(name));
}

bool ChaiEnv::object_cur_position(std::string name, double px, double py, double pz){
    m_objectIt = m_objectMap.find(name);
    if(m_objectIt != m_objectMap.end()){
        m_objectMap[name]->cur_position(px, py, pz);
        return true;
    }
    else{
        return false;
    }
}

bool ChaiEnv::object_cur_orientation(std::string name, double roll, double pitch, double yaw){
    m_objectIt = m_objectMap.find(name);
    if(m_objectIt != m_objectMap.end()){
        m_objectMap[name]->cur_orientation(roll, pitch, yaw);
        return true;
    }
    else{
        return false;
    }
}

ChaiEnv::~ChaiEnv(){

}
