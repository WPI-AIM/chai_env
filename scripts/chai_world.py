#!/usr/bin/env python
from chai_msgs.msg import WorldState, WorldCmd
from watch_dog import WatchDog


class World(WatchDog):
    def __init__(self, a_name, a_objects_dict):
        super(World, self).__init__()
        self.m_state = WorldState()
        self.m_objects_dict = a_objects_dict
        self.m_name = a_name
        self.m_cmd = WorldCmd()
        self.m_cmd.enable_step_throttling = False
        self.m_pub = None
        self.m_sub = None
        self.m_pub_flag = True

    def enable_throttling(self, data):
        self.m_cmd.enable_step_throttling = data

    def ros_cb(self, data):
        self.m_state = data

    def check_objects_step(self):
        step = False
        if self.m_cmd.enable_step_throttling is True:
            for key, obj in self.m_objects_dict.iteritems():
                if key == 'World' or key == 'world':
                    pass
                else:
                    if obj.m_step_flag is True:
                        step = True
                        obj.m_step_flag = False
        if step is True:
            self.m_cmd.step = not self.m_cmd.step
        self.acknowledge_wd()

    def clear_cmd(self):
        self.m_cmd.enable_step_throttling = False
        pass

    def run_publisher(self):
        self.check_objects_step()
        if self.m_pub_flag:
            if self.is_wd_expired():
                self.clear_cmd()
            self.m_pub.publish(self.m_cmd)