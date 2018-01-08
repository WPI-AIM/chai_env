#include "chai_env/World.h"

World::World(std::string a_name): WorldRosCom(a_name){

}

void World::set_chai_wall_time(double a_sec){
    m_worldState.chai_wall_time = a_sec;
    increment_sim_step();
    m_worldState.header.stamp = ros::Time::now();
}

void World::set_chai_sim_time(double a_sec){
    m_worldState.chai_sim_time = a_sec;
}

void World::increment_sim_step(){
    m_worldState.sim_step++;
}

extern "C"{

World* create_world(std::string name){
    return new World(name);
}

void destroy_world(World* obj){
    delete obj;
}

}
