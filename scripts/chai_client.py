import rospy
from chai_msg.msg import ObjectState
from threading import Lock

class Object:
    def __init__(self, a_name):
        self.header = []
        self.name = a_name
        self.pose = []



class ChaiClient:
    def __init__(self):
        self.active_topics = []
        self.search_string_prefix = '/chai/env'
        self.search_string_end = '/State'
        self.m_nObjects = 0
        self.m_sub_list = []
        self.m_obj_list = []
        self.mutex = Lock()
        pass

    def process_topics(self):
        rospy.init_node('chai_client')
        self.active_topics = rospy.get_published_topics()

        pass

    def print_active_topics(self):
        print self.active_topics
        pass

    def obj_sub_cb(self, data, a_objInd):
        self.mutex.acquire()
        print 'Object Data ', data.name, ' Object Index', a_objInd
        self.mutex.release()

    def find_objects(self):
        for i in range(len(self.active_topics)):
            for j in range(len(self.active_topics[i])):
                print 'At: ', i, j, self.active_topics[i][j]
                prefix_ind = self.active_topics[i][j].find(self.search_string_prefix)
                if prefix_ind >= 0:
                    search_ind = self.active_topics[i][j].find(self.search_string_end)
                    if search_ind >=0:
                        self.m_obj_list.append(Object)
                        objInd = self.m_nObjects
                        self.m_sub_list.append(rospy.Subscriber(self.active_topics[i][j],
                                                                ObjectState, self.obj_sub_cb,
                                                                callback_args=objInd))
                        self.m_nObjects += 1
                        print 'Found state', self.active_topics[i][j], ' No of Objects = ', self.m_nObjects


def main():
    clientObj = ChaiClient()
    clientObj.process_topics()
    clientObj.find_objects()
    rospy.spin()

if __name__=='__main__':
    main()
