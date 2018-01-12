#!/usr/bin/env python
from tf import transformations
from geometry_msgs.msg import Pose
from chai_msgs.msg import ObjectCmd
from watch_dog import WatchDog
import rospy


class Object(WatchDog):
    def __init__(self, a_name):
        super(Object, self).__init__()
        self.time_stamp = []
        self.sim_step_cur = 0
        self.sim_step_pre_update = 0
        self.name = a_name
        self.pose = Pose()
        self.cmd = ObjectCmd()
        self.pub = None
        self.sub = None
        self.pub_flag = True
        self.step_sim_flag = False

    def ros_cb(self, data):
        self.name = data.name.data
        self.pose = data.pose
        self.time_stamp = data.header.stamp
        self.sim_step_cur = data.sim_step

    def command(self, fx, fy, fz, nx, ny, nz, t1=0, t2=0, t3=0):
        self.cmd.wrench.force.x = fx
        self.cmd.wrench.force.y = fy
        self.cmd.wrench.force.z = fz
        self.cmd.wrench.torque.x = nx
        self.cmd.wrench.torque.x = ny
        self.cmd.wrench.torque.x = nz
        self.cmd.joint_cmds[0] = t1
        self.cmd.joint_cmds[1] = t2
        self.cmd.joint_cmds[2] = t3
        self.cmd.header.stamp = rospy.Time.now()
        self.acknowledge_wd()

    def set_sim_step_flag(self):
        self.step_sim_flag = True
        self.sim_step_pre_update = self.sim_step_cur

    def get_sim_step_flag(self):
        return self.step_sim_flag

    def get_cur_sim_step(self):
        return self.sim_step_cur

    def get_pre_update_sim_step(self):
        return self.sim_step_pre_update

    def clear_cmd(self):
        self.cmd.wrench.force.x = 0
        self.cmd.wrench.force.y = 0
        self.cmd.wrench.force.z = 0
        self.cmd.wrench.torque.x = 0
        self.cmd.wrench.torque.x = 0
        self.cmd.wrench.torque.x = 0

    def get_pose(self):
        quat = self.pose.orientation
        explicit_quat = [quat.x, quat.y, quat.z, quat.w]
        rpy = transformations.euler_from_quaternion(explicit_quat, 'szyx')
        pose = [self.pose.position.x,
                self.pose.position.y,
                self.pose.position.z,
                rpy[0],
                rpy[1],
                rpy[2]]
        return pose

    def run_publisher(self):
        if self.pub_flag:
            if self.is_wd_expired():
                self.clear_cmd()
            self.pub.publish(self.cmd)