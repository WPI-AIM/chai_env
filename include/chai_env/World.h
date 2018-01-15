#ifndef WORLD_H
#define WORLD_H

#include <string>
#include "chai_env/WorldRosCom.h"

namespace chai_env{
class World: public WorldRosCom{
public:
    World(std::string a_name);
    inline void set_chai_wall_time(double a_sec);
    inline void set_chai_sim_time(double a_sec);
    inline void increment_sim_step();
    inline void set_num_devices(uint a_num);
    inline void set_loop_freq(double a_freq);

    ////////////////////////////
    inline bool step_sim();
};
}

#endif
