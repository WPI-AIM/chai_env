#ifndef WORLD_H
#define WORLD_H

#include <string>
#include "chai_env/WorldRosCom.h"

class World: public WorldRosCom{
public:
    World(std::string a_name);
    void set_chai_wall_time(double a_sec);
    void set_chai_sim_time(double a_sec);
    void increment_sim_step();
};


#endif
