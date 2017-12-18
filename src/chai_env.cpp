#include "chai_env/chai_env.h"
#include <string>

ChaiEnv::ChaiEnv(){
    m_numObjects = 0;
}


void ChaiEnv::add_object(std::string name){
    m_objectIt = m_objectMap.find(name);
    if(m_objectIt == m_objectMap.end()){
        m_objectMap[name] = boost::shared_ptr<Object>(new Object(name));
    }
    else{
        std::cerr<< "ERROR!, OBJECT: \""<< name << "\" ALREADY EXISTS. IGNORING" << std::endl;
    }
}

bool ChaiEnv::object_cur_position(std::string name, double px, double py, double pz){
    m_objectIt = m_objectMap.find(name);
    if(m_objectIt != m_objectMap.end()){
        m_objectMap[name]->cur_position(px, py, pz);
        return true;
    }
    else{
        std::cerr<< "ERROR!, OBJECT: \""<< name << "\" DOESN'T EXIST" << std::endl;
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
        std::cerr<< "ERROR!, OBJECT: \""<< name << "\" DOESN'T EXIST" << std::endl;
        return false;
    }
}

ChaiEnv::~ChaiEnv(){

}
