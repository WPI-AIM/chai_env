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

#ifndef ROSCOMBASE_H
#define ROSCOMBASE_H

#include "chai_env/StaticParams.h"
#include "chai_env/CmdWatchDog.h"


template <class T_state, class T_cmd>
class RosComBase: public StaticParams{
public:
    RosComBase(std::string a_name, int a_freq_min = 10, int a_freq_max = 2500)
    {
        m_name = a_name;
        aspinPtr.reset(new ros::AsyncSpinner(5));
        nodePtr->setCallbackQueue(&m_custom_queue);
        m_watchDogPtr.reset(new CmdWatchDog(a_freq_min, a_freq_max));
    }
    virtual void init() = 0;
    virtual void run_publishers();

protected:
    boost::shared_ptr<ros::AsyncSpinner> aspinPtr;
    boost::shared_ptr<CmdWatchDog> m_watchDogPtr;

    std::string m_name;
    ros::Publisher m_pub;
    ros::Subscriber m_sub;

    tf::Transform m_trans;
    T_state m_State;
    T_cmd m_Cmd;
    T_cmd m_CmdPrev;

    boost::thread m_thread;
    ros::CallbackQueue m_custom_queue;

    virtual void reset_cmd() = 0;
};


//template<class T_state, class T_cmd>
//void RosComBase<T_state, T_cmd>::run_publishers(){
//    while(nodePtr->ok()){
//        m_pub.publish(m_State);
//        m_custom_queue.callAvailable();
//        if(m_watchDogPtr->is_wd_expired()){
//            m_watchDogPtr->consolePrint(m_name);
//            reset_cmd();
//        }
//        m_watchDogPtr->m_ratePtr->sleep();
//    }
//}


#endif
