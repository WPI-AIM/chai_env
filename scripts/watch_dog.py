#!/usr/bin/env python
import rospy


class WatchDog(object):
    def __init__(self, time_out = 0.1):
        self.m_expire_duration = rospy.Duration.from_sec(time_out)
        self.m_next_cmd_expected_time = rospy.Time.now()

    def acknowledge_wd(self):
        self.m_next_cmd_expected_time = rospy.Time.now() + self.m_expire_duration

    def is_wd_expired(self):
        if rospy.Time.now() > self.m_next_cmd_expected_time:
            return True
        else:
            return False
