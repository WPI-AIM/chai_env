import rospy
from chai_msg.msg import ObjectState
from threading import Lock
import threading
from geometry_msgs.msg import WrenchStamped, Pose
from tf import transformations


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


class Object(WatchDog):
    def __init__(self, a_name):
        super(Object, self).__init__()
        self.m_time_stamp = []
        self.m_name = a_name
        self.m_pose = Pose()
        self.m_cmd = WrenchStamped()
        self.m_pub = threading.Thread()
        self.m_sub = threading.Thread()
        self.m_pub_flag = True

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
        self.acknowledge_wd()

    def clear_cmd(self):
        self.m_cmd.wrench.force.x = 0
        self.m_cmd.wrench.force.y = 0
        self.m_cmd.wrench.force.z = 0
        self.m_cmd.wrench.torque.x = 0
        self.m_cmd.wrench.torque.x = 0
        self.m_cmd.wrench.torque.x = 0

    def get_pose(self):
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


class ChaiClient:
    def __init__(self):
        self.m_ros_topics = []
        self.m_search_prefix_str = '/chai/env/'
        self.m_search_suffix_str = '/State'
        self.m_string_cmd = '/Command'
        self.m_sub_list = []
        self.m_mutex = Lock()
        self.m_objects_dict = {}
        self.m_sub_thread = []
        self.m_pub_thread = []
        self.m_rate = 0
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
        self.start_pubs()

    def start_subs(self):
        self.m_sub_thread = threading.Thread(target=rospy.spin)
        self.m_sub_thread.start()

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

    def create_objs_from_rostopics(self):
        rospy.init_node('chai_client')
        rospy.on_shutdown(self.clean_up)
        self.m_rate = rospy.Rate(1000)
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
                        obj = Object(obj_name)
                        obj.m_sub = rospy.Subscriber(self.m_ros_topics[i][j],
                                                     ObjectState,
                                                     obj.ros_cb)

                        pub_topic_str = self.m_search_prefix_str + obj.m_name + self.m_string_cmd
                        obj.m_pub = rospy.Publisher(name=pub_topic_str, data_class=WrenchStamped, queue_size=10)
                        self.m_objects_dict[obj_name] = obj

    def clean_up(self):
        for key, val in self.m_objects_dict.iteritems():
            val.m_pub_flag = False
            print 'Closing publisher for: ', key

