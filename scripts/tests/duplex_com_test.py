from chai_msg.msg import WorldCmd, WorldState
import rospy

cmd = WorldCmd()
global sim_step, sim_step_prev
sim_step = 0
sim_step_prev = 0



def cb(data):
    global sim_step, sim_step_prev
    sim_step_prev = sim_step
    sim_step = data.sim_step
    print sim_step - sim_step_prev
    pass


def main():
    rospy.init_node('duplex_comm_text')
    sub = rospy.Subscriber('/chai/env/World/State', WorldState, callback=cb , queue_size=10)
    pub = rospy.Publisher('/chai/env/World/Command', WorldCmd, queue_size=10)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        cmd.enable_step_throttling = True
        cmd.step = not cmd.step
        pub.publish(cmd)
        rate.sleep()


if __name__ == '__main__':
    main()