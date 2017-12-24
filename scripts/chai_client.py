import rospy
from chai_msg.msg import ObjectState
from threading import Lock
import threading
from geometry_msgs.msg import WrenchStamped, Pose
import time


class Object:
    def __init__(self, a_name):
        self.m_time_stamp = []
        self.m_name = a_name
        self.m_pose = Pose()
        self.m_cmd = WrenchStamped()
        self.m_pub = []
        self.m_sub = []

    def ros_cb(self, data):
        self.m_name = data.name.data
        self.m_pose = data.pose_cur
        self.m_time_stamp = data.header.stamp

    def command(self, fx, fy, fz, nx, ny, nz):
        self.m_cmd.wrench.force.x = fx
        self.m_cmd.wrench.force.y = fy
        self.m_cmd.wrench.force.z = fz
        self.m_cmd.wrench.torque.x = nx
        self.m_cmd.wrench.torque.x = ny
        self.m_cmd.wrench.torque.x = nz

    def run_publisher(self):
        self.m_pub.publish(self.m_cmd)


class ChaiClient:
    def __init__(self):
        self.m_ros_topics = []
        self.m_search_string_prefix = '/chai/env/'
        self.m_search_string_end = '/State'
        self.m_string_cmd = '/Command'
        self.m_sub_list = []
        self.m_mutex = Lock()
        self.m_objects_dict = {}
        pass

    def get_obj_pose(self, a_name):
        obj = self.m_objects_dict.get(a_name)
        if obj is not None:
            return obj.m_pose
        else:
            return None

    def get_obj_handle(self, a_name):
        obj = self.m_objects_dict.get(a_name)
        return obj

    def set_obj_cmd(self, a_name, fx, fy, fz, nx, ny, nz):
        obj = self.m_objects_dict.get(a_name)
        obj.command(fx, fy, fz, nx, ny, nz)

    def start(self):
        self.start_subs()
        self.start_pubs()

    def start_subs(self):
        tsub = threading.Thread(target=rospy.spin)
        tsub.start()

    def start_pubs(self):
        tpub = threading.Thread(target=self.run_obj_publishers)
        tpub.start()

    def run_obj_publishers(self):
        while not rospy.is_shutdown():
            for key, obj in self.m_objects_dict.items():
                obj.run_publisher()
            rospy.sleep(0.001)

    def print_active_topics(self):
        print self.m_ros_topics
        pass

    def print_summary(self):
        print 'Objects are: '
        for key, value in self.m_objects_dict.items():
            print key

    def create_objs_from_rostopics(self):
        rospy.init_node('chai_client')
        self.m_ros_topics = rospy.get_published_topics()
        for i in range(len(self.m_ros_topics)):
            for j in range(len(self.m_ros_topics[i])):
                prefix_ind = self.m_ros_topics[i][j].find(self.m_search_string_prefix)
                if prefix_ind >= 0:
                    search_ind = self.m_ros_topics[i][j].find(self.m_search_string_end)
                    if search_ind >= 0:
                        # Searching the active topics between the end of prefix:/chai/env/ and start of /State
                        obj_name = self.m_ros_topics[i][j][
                                     prefix_ind + len(self.m_search_string_prefix):search_ind]
                        obj = Object(obj_name)
                        obj.m_sub = rospy.Subscriber(self.m_ros_topics[i][j],
                                                     ObjectState,
                                                     obj.ros_cb)

                        pub_topic_str = self.m_search_string_prefix + obj.m_name + self.m_string_cmd
                        obj.m_pub = rospy.Publisher(name=pub_topic_str, data_class=WrenchStamped, queue_size=10)
                        self.m_objects_dict[obj_name] = obj



def main():
    clientObj = ChaiClient()
    clientObj.create_objs_from_rostopics()
    clientObj.print_summary()
    clientObj.start()
    while not rospy.is_shutdown():
        obj = clientObj.get_obj_handle('Torus')
        obj.command(0,0,0,0,0,0)
        print obj.m_name
        print obj.m_pose.position
        rospy.sleep(0.1)


if __name__=='__main__':
    main()
