#!/usr/bin/env python
from tf import transformations
from geometry_msgs.msg import Pose
from chai_msgs.msg import ObjectCmd
from watch_dog import WatchDog
import rospy


class Object(WatchDog):
    def __init__(self, a_name):
        super(Object, self).__init__()
        self.m_time_stamp = []
        self.m_sim_step = 0
        self.m_sim_step_prev = 0
        self.m_name = a_name
        self.m_pose = Pose()
        self.m_cmd = ObjectCmd()
        self.m_pub = None
        self.m_sub = None
        self.m_pub_flag = True
        self.m_step = False

    def ros_cb(self, data):
        self.m_name = data.name.data
        self.m_pose = data.pose
        self.m_time_stamp = data.header.stamp
        self.m_sim_step = data.sim_step

    def command(self, fx, fy, fz, nx, ny, nz, t1=0, t2=0, t3=0):
        self.m_cmd.wrench.force.x = fx
        self.m_cmd.wrench.force.y = fy
        self.m_cmd.wrench.force.z = fz
        self.m_cmd.wrench.torque.x = nx
        self.m_cmd.wrench.torque.x = ny
        self.m_cmd.wrench.torque.x = nz
        self.m_cmd.joint_cmds[0] = t1
        self.m_cmd.joint_cmds[1] = t2
        self.m_cmd.joint_cmds[2] = t3
        self.m_cmd.header.stamp = rospy.Time.now()
        self.m_sim_step_prev = self.m_sim_step
        self.m_step = True
        self.acknowledge_wd()

    def clear_cmd(self):
        self.m_cmd.wrench.force.x = 0
        self.m_cmd.wrench.force.y = 0
        self.m_cmd.wrench.force.z = 0
        self.m_cmd.wrench.torque.x = 0
        self.m_cmd.wrench.torque.x = 0
        self.m_cmd.wrench.torque.x = 0

    def get_pose(self):
        if not self.m_sim_step > self.m_sim_step_prev:
            return 0

        step_jump = self.m_sim_step - self.m_sim_step_prev
        if step_jump > 1:
            print 'WARN: {} steps jumped'.format(step_jump)
        quat = self.m_pose.orientation
        explicit_quat = [quat.x, quat.y, quat.z, quat.w]
        rpy = transformations.euler_from_quaternion(explicit_quat, 'szyx')
        pose = [self.m_pose.position.x,
                self.m_pose.position.y,
                self.m_pose.position.z,
                rpy[0],
                rpy[1],
                rpy[2]]
        return pose

    def run_publisher(self):
        if self.m_pub_flag:
            if self.is_wd_expired():
                self.clear_cmd()
            self.m_pub.publish(self.m_cmd)