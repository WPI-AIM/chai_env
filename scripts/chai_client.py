import rospy
from chai_msg.msg import ObjectState
from threading import Lock

class Object:
    def __init__(self, a_name):
        self.m_header = []
        self.m_name = a_name
        self.m_pose = []



class ChaiClient:
    def __init__(self):
        self.m_active_topics = []
        self.m_search_string_prefix = '/chai/env'
        self.m_search_string_end = '/State'
        self.m_nObjects = 0
        self.m_sub_list = []
        self.m_obj_list = []
        self.m_mutex = Lock()
        pass

    def process_topics(self):
        rospy.init_node('chai_client')
        self.m_active_topics = rospy.get_published_topics()

        pass

    def print_active_topics(self):
        print self.m_active_topics
        pass

    def obj_sub_cb(self, data, a_objInd):
        self.m_mutex.acquire()
        print 'Object Data ', data.name, ' Object Index', a_objInd
        self.m_mutex.release()

    def find_objects(self):
        for i in range(len(self.m_active_topics)):
            for j in range(len(self.m_active_topics[i])):
                print 'At: ', i, j, self.m_active_topics[i][j]
                prefix_ind = self.m_active_topics[i][j].find(self.m_search_string_prefix)
                if prefix_ind >= 0:
                    search_ind = self.m_active_topics[i][j].find(self.m_search_string_end)
                    if search_ind >=0:
                        self.m_obj_list.append(Object)
                        objInd = self.m_nObjects
                        self.m_sub_list.append(rospy.Subscriber(self.m_active_topics[i][j],
                                                                ObjectState, self.obj_sub_cb,
                                                                callback_args=objInd))
                        self.m_nObjects += 1
                        print 'Found state', self.m_active_topics[i][j], ' No of Objects = ', self.m_nObjects


def main():
    clientObj = ChaiClient()
    clientObj.process_topics()
    clientObj.find_objects()
    rospy.spin()

if __name__=='__main__':
    main()
