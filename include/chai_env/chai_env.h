#ifndef CHAI_ENV_H
#define CHAI_ENV_H
#include <ros/ros.h>
#include "chai_env/Object.h"
#include <tf/LinearMath/Transform.h>

struct Observation{
public:
    Observation();

    geometry_msgs::PoseStamped m_nextState;
    double m_reward;
    double m_done;
};

class ChaiEnv{
public:
    ChaiEnv();
    ~ChaiEnv();
    void add_object(std::string name);
    chai_env::Object* get_object_handle(std::string name);
    bool object_cur_position(std::string name, double px, double py, double pz);
    bool object_cur_orientation(std::string name, double roll, double pitch, double yaw);
    bool object_cur_force(std::string name, double fx, double fy, double fz);
    bool object_cur_torque(std::string name, double nx, double ny, double nz);

private:

    bool object_exists(std::string name);
    static const int max_obj_size=10;
    int m_numObjects;
    std::map<std::string, boost::shared_ptr<chai_env::Object> > m_objectMap;
    std::map<std::string, boost::shared_ptr<chai_env::Object> >::iterator m_objectIt;
    boost::shared_ptr<chai_env::Object> m_Objects[max_obj_size];
};


#endif
