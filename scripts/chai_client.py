#!/usr/bin/env python
import rospy
from chai_msgs.msg import ObjectState, ObjectCmd, WorldState, WorldCmd
import threading
from geometry_msgs.msg import WrenchStamped
from chai_object import Object
from chai_world import World


class ChaiClient:
    def __init__(self):
        self.m_ros_topics = []
        self.m_search_prefix_str = '/chai/env/'
        self.m_search_suffix_str = '/State'
        self.m_string_cmd = '/Command'
        self.m_sub_list = []
        self.m_objects_dict = {}
        self.m_sub_thread = []
        self.m_pub_thread = []
        self.m_rate = 0
        self.m_world_name = ''
        pass

    def create_objs_from_rostopics(self):
        rospy.init_node('chai_client')
        rospy.on_shutdown(self.clean_up)
        self.m_rate = rospy.Rate(3000)
        self.m_ros_topics = rospy.get_published_topics()
        for i in range(len(self.m_ros_topics)):
            for j in range(len(self.m_ros_topics[i])):
                prefix_ind = self.m_ros_topics[i][j].find(self.m_search_prefix_str)
                if prefix_ind >= 0:
                    search_ind = self.m_ros_topics[i][j].find(self.m_search_suffix_str)
                    if search_ind >= 0:
                        # Searching the active topics between the end of prefix:/chai/env/ and start of /State
                        obj_name = self.m_ros_topics[i][j][
                                     prefix_ind + len(self.m_search_prefix_str):search_ind]
                        if obj_name == 'World' or obj_name == 'world':
                            self.m_world_name = obj_name
                            obj = World(obj_name, self.m_objects_dict)
                            obj.m_sub = rospy.Subscriber(self.m_ros_topics[i][j], WorldState, obj.ros_cb)
                            pub_topic_str = self.m_search_prefix_str + obj.m_name + self.m_string_cmd
                            obj.m_pub = rospy.Publisher(name=pub_topic_str, data_class=WorldCmd, queue_size=10)
                        else:
                            obj = Object(obj_name)
                            obj.m_sub = rospy.Subscriber(self.m_ros_topics[i][j], ObjectState, obj.ros_cb)
                            pub_topic_str = self.m_search_prefix_str + obj.m_name + self.m_string_cmd
                            obj.m_pub = rospy.Publisher(name=pub_topic_str, data_class=ObjectCmd, queue_size=10)

                        self.m_objects_dict[obj_name] = obj

    def start(self):
        self.start_pubs()

    def get_obj_handle(self, a_name):
        obj = self.m_objects_dict.get(a_name)
        return obj

    def get_obj_pose(self, a_name):
        obj = self.m_objects_dict.get(a_name)
        if obj is not None:
            return obj.m_pose
        else:
            return None

    def enable_throttling(self, data):
        if self.m_world_name:
            self.m_objects_dict[self.m_world_name].enable_throttling(data)
        else:
            raise Exception

    def set_obj_cmd(self, a_name, fx, fy, fz, nx, ny, nz):
        obj = self.m_objects_dict.get(a_name)
        obj.command(fx, fy, fz, nx, ny, nz)

    def start_pubs(self):
        self.m_pub_thread = threading.Thread(target=self.run_obj_publishers)
        self.m_pub_thread.daemon = True
        self.m_pub_thread.start()

    def run_obj_publishers(self):
        while not rospy.is_shutdown():
            for key, obj in self.m_objects_dict.items():
                obj.run_publisher()
            self.m_rate.sleep()

    def print_active_topics(self):
        print self.m_ros_topics
        pass

    def print_summary(self):
        print '_________________________________________________________'
        print '---------------------------------------------------------'
        print 'CLIENT FOR CREATING OBJECTS FROM ROSTOPICS'
        print 'Searching Object names from ros topics with'
        print 'Prefix: ', self.m_search_prefix_str
        print 'Suffix: ', self.m_search_suffix_str
        print 'Number of OBJECTS found', len(self.m_objects_dict)
        for key, value in self.m_objects_dict.items():
            print key
        print '---------------------------------------------------------'

    def clean_up(self):
        for key, val in self.m_objects_dict.iteritems():
            val.m_pub_flag = False
            print 'Closing publisher for: ', key

