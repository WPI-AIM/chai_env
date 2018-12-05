//===========================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2017-2018

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    \author    amunawar@wpi.edu
    \author    Adnan Munawar
    \version   0.1
*/
//===========================================================================

#include "chai_env/World.h"

namespace chai_env{

World::World(std::string a_name): World(a_name, "/chai/env/"){
}

World::World(std::string a_name, std::string a_namespace): WorldRosCom(a_name, a_namespace){
    m_num_skip_steps = 10;
    m_skip_steps_ctr = 0;
}

void World::set_chai_wall_time(double a_sec){
    m_State.chai_wall_time = a_sec;
    increment_sim_step();
    m_State.header.stamp = ros::Time::now();
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
    m_State.sim_step++;
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
