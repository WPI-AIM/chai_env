#include "chai_env/World.h"

namespace chai_env{
World::World(std::string a_name): WorldRosCom(a_name){
    m_num_skip_steps = 10;
    m_skip_steps_ctr = 0;
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
        m_skip_steps_ctr++;
        if (m_skip_steps_ctr == m_num_skip_steps){
            m_stepSim = false;
            m_skip_steps_ctr = 0;
        }
        if (m_skip_steps_ctr > m_num_skip_steps){
            std::cerr << "WARN, Skipped " << m_skip_steps_ctr << " steps, Default skip limit " << m_num_skip_steps << std::endl;
        }
    }
    m_worldState.sim_step++;
}

void World::set_num_devices(uint a_num){
    m_worldState.n_devices = a_num;
}

void World::set_loop_freq(double a_freq){
    m_worldState.dynamic_loop_freq = a_freq;
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
