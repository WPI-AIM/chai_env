from chai_msg.msg import WorldCmd, WorldState
import rospy

cmd = WorldCmd()


def cb(data):
    pass


def main():
    rospy.init_node('duplex_comm_text')
    sub = rospy.Subscriber('/chai/env/World/State', WorldState, queue_size=10)
    pub = rospy.Publisher('/chai/env/World/Command', WorldCmd, queue_size=10)
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        cmd.enable_step_throttling = True
        cmd.step = not cmd.step
        pub.publish(cmd)
        rate.sleep()


if __name__ == '__main__':
    main()