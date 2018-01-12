#include "chai_env/World.h"

namespace chai_env{
World::World(std::string a_name): WorldRosCom(a_name){
    n_skip_steps = 10;
    skip_steps_ctr = 0;
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
    if(m_enableSimThrottle){
        skip_steps_ctr++;
        if (skip_steps_ctr == n_skip_steps){
            m_stepSim = false;
            skip_steps_ctr = 0;
        }
        if (skip_steps_ctr > n_skip_steps){
            std::cerr << "WARN, Skipped " << skip_steps_ctr << " steps, Default skip limit " << n_skip_steps << std::endl;
        }
    }
    m_worldState.sim_step++;
}

bool World::step_sim(){
    return m_stepSim;
}

extern "C"{

World* create_world(std::string name){
    return new World(name);
}

void destroy_world(World* obj){
    delete obj;
}

}
}
