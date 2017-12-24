import rospy
from chai_msg.msg import ObjectState
from threading import Lock, Thread
from geometry_msgs.msg import WrenchStamped


class Object:
    def __init__(self, a_name):
        self.m_time_stamp = []
        self.m_name = a_name
        self.m_pose = []
        self.m_cmd = WrenchStamped()
        self.m_pub = []
        self.m_sub = []

    def ros_cb(self, data):
        self.m_name = data.name.data
        self.m_pose = data.pose_cur
        self.m_time_stamp = data.header.stamp

    def command(self, cmd):
        self.m_cmd.wrench = cmd

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

    def process_topics(self):
        rospy.init_node('chai_client')
        self.m_ros_topics = rospy.get_published_topics()
        pass

    def print_active_topics(self):
        print self.m_ros_topics
        pass

    def print_summary(self):
        print 'Objects are: '
        for key, value in self.m_objects_dict.iteritems():
            print key

    def find_objects(self):
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
    clientObj.process_topics()
    clientObj.find_objects()
    clientObj.print_summary()
    rospy.spin()

if __name__=='__main__':
    main()
