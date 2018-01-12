from chai_msgs.msg import WorldCmd, WorldState
import rospy

cmd = WorldCmd()
global cb_ctr
cb_ctr = 0


def cb(data):
    global cb_ctr
    cb_ctr +=1
    if cb_ctr % 100 == 0:
        print data.sim_step


def main():
    rospy.init_node('duplex_comm_text')
    sub = rospy.Subscriber('/chai/env/World/State', WorldState, callback=cb , queue_size=10)
    pub = rospy.Publisher('/chai/env/World/Command', WorldCmd, queue_size=10)
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        cmd.enable_step_throttling = True
        cmd.step_clock = not cmd.step_clock
        cmd.n_skip_steps = 100
        pub.publish(cmd)
        rate.sleep()


if __name__ == '__main__':
    main()