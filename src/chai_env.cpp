#include "chai_env/chai_env.h"
#include <string>

ChaiEnv::ChaiEnv(){
    m_numObjects = 0;
}


void ChaiEnv::add_object(std::string name){
    if(! object_exists(name)){
        m_objectMap[name] = boost::shared_ptr<Object>(new Object(name));
    }
    else{
        std::cerr<< "ERROR!, OBJECT: \""<< name << "\" ALREADY EXISTS. IGNORING" << std::endl;
    }
}

Object* ChaiEnv::get_object_handle(std::string name){
    if(object_exists(name)){
        return m_objectMap[name].get();
    }
    else{
        std::cerr<< "ERROR!, OBJECT: \""<< name << "\" DOESN'T EXIST" << std::endl;
        return NULL;
    }
}

bool ChaiEnv::object_exists(std::string name){
    m_objectIt = m_objectMap.find(name);
    if(m_objectIt != m_objectMap.end()){
        return true;
    }
    else{
        return false;
    }
}

bool ChaiEnv::object_cur_position(std::string name, double px, double py, double pz){
    if(object_exists(name)){
        m_objectMap[name]->cur_position(px, py, pz);
        return true;
    }
    else{
        std::cerr<< "ERROR!, OBJECT: \""<< name << "\" DOESN'T EXIST" << std::endl;
        return false;
    }
}

bool ChaiEnv::object_cur_orientation(std::string name, double roll, double pitch, double yaw){
    if(object_exists(name)){
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
