#ifndef STATICPARAMS_H
#define STATICPARAMS_H

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <tf/tf.h>
#include <tf/LinearMath/Transform.h>
#include <ros/callback_queue.h>
#include <ros/duration.h>
#include <iostream>

class StaticParams{
public:
    StaticParams();
    static boost::shared_ptr<ros::NodeHandle> nodePtr;
    static std::string m_chai_namespace;
    static int m_usage_ctr;
};

#endif
