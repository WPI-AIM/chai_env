#!/usr/bin/env python
import rospy
from chai_msgs.msg import ObjectState, ObjectCmd, WorldState, WorldCmd
import threading
from geometry_msgs.msg import WrenchStamped
from chai_object import Object
from chai_world import World


class ChaiClient:
    def __init__(self):
        self.ros_topics = []
        self.search_prefix_str = '/chai/env/'
        self.search_suffix_str = '/State'
        self.string_cmd = '/Command'
        self.sub_list = []
        self.objects_dict = {}
        self.sub_thread = []
        self.pub_thread = []
        self.rate = 0
        self.world_name = ''
        pass

    def create_objs_from_rostopics(self):
        rospy.init_node('chai_client')
        rospy.on_shutdown(self.clean_up)
        self.rate = rospy.Rate(1000)
        self.ros_topics = rospy.get_published_topics()
        for i in range(len(self.ros_topics)):
            for j in range(len(self.ros_topics[i])):
                prefix_ind = self.ros_topics[i][j].find(self.search_prefix_str)
                if prefix_ind >= 0:
                    search_ind = self.ros_topics[i][j].find(self.search_suffix_str)
                    if search_ind >= 0:
                        # Searching the active topics between the end of prefix:/chai/env/ and start of /State
                        obj_name = self.ros_topics[i][j][
                                     prefix_ind + len(self.search_prefix_str):search_ind]
                        if obj_name == 'World' or obj_name == 'world':
                            self.world_name = obj_name
                            obj = World(obj_name)
                            obj.sub = rospy.Subscriber(self.ros_topics[i][j], WorldState, obj.ros_cb)
                            pub_topic_str = self.search_prefix_str + obj.name + self.string_cmd
                            obj.pub = rospy.Publisher(name=pub_topic_str, data_class=WorldCmd, queue_size=10)
                        else:
                            obj = Object(obj_name)
                            obj.sub = rospy.Subscriber(self.ros_topics[i][j], ObjectState, obj.ros_cb)
                            pub_topic_str = self.search_prefix_str + obj.name + self.string_cmd
                            obj.pub = rospy.Publisher(name=pub_topic_str, data_class=ObjectCmd,tcp_nodelay=True, queue_size=10)

                        self.objects_dict[obj_name] = obj

    def start(self):
        self.start_pubs()

    def get_obj_handle(self, a_name):
        obj = self.objects_dict.get(a_name)
        obj.set_active()
        return obj

    def get_obj_pose(self, a_name):
        obj = self.objects_dict.get(a_name)
        if obj is not None:
            return obj.pose
        else:
            return None

    def set_obj_cmd(self, a_name, fx, fy, fz, nx, ny, nz):
        obj = self.objects_dict.get(a_name)
        obj.command(fx, fy, fz, nx, ny, nz)

    def start_pubs(self):
        self.pub_thread = threading.Thread(target=self.run_obj_publishers)
        self.pub_thread.daemon = True
        self.pub_thread.start()

    def run_obj_publishers(self):
        while not rospy.is_shutdown():
            for key, obj in self.objects_dict.items():
                if obj.is_active():
                    obj.run_publisher()
            self.rate.sleep()

    def print_active_topics(self):
        print self.ros_topics
        pass

    def print_summary(self):
        print '_________________________________________________________'
        print '---------------------------------------------------------'
        print 'CLIENT FOR CREATING OBJECTS FROM ROSTOPICS'
        print 'Searching Object names from ros topics with'
        print 'Prefix: ', self.search_prefix_str
        print 'Suffix: ', self.search_suffix_str
        print 'Number of OBJECTS found', len(self.objects_dict)
        for key, value in self.objects_dict.items():
            print key
        print '---------------------------------------------------------'

    def clean_up(self):
        for key, val in self.objects_dict.iteritems():
            val.pub_flag = False
            print 'Closing publisher for: ', key

