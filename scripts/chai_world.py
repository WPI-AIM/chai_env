#!/usr/bin/env python
from chai_msgs.msg import WorldState, WorldCmd
from watch_dog import WatchDog


class World(WatchDog):
    def __init__(self, a_name, a_objects_dict):
        super(World, self).__init__()
        self.state = WorldState()
        self.objects_dict = a_objects_dict
        self.name = a_name
        self.cmd = WorldCmd()
        self.cmd.enable_step_throttling = False
        self.pub = None
        self.sub = None
        self.pub_flag = True

    def enable_throttling(self, data):
        self.cmd.enable_step_throttling = data

    def ros_cb(self, data):
        self.state = data

    def update(self):
        self.cmd.step_clock = not self.cmd.step_clock
        self.acknowledge_wd()

    def clear_cmd(self):
        self.cmd.enable_step_throttling = False
        pass

    def run_publisher(self):
        if self.pub_flag:
            if self.is_wd_expired():
                self.clear_cmd()
            self.pub.publish(self.cmd)