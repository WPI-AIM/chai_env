#include <chai_env/RosComBase.h>


template<class T_state, class T_cmd>
void RosComBase<T_state, T_cmd>::run_publishers(){
    while(nodePtr->ok()){
        m_pub.publish(m_State);
        m_custom_queue.callAvailable();
        if(m_watchDogPtr->is_wd_expired()){
            m_watchDogPtr->consolePrint(m_name);
            reset_cmd();
        }
        m_watchDogPtr->m_ratePtr->sleep();
    }
}
