import rospy
from chai_msg.msg import ObjectState
import matplotlib.pyplot as plt
import rospy.rostime as time


sim_time = []
wall_time = []
cur_time = []
itrs = []
global i
i=0
global first_run, time_adjust
first_run = True


def msg_cb(data):
    global i, first_run, time_adjust
    if first_run and data.header.stamp.to_sec() > 0.1:
        time_adjust = rospy.Time.now().to_sec() - data.header.stamp.to_sec()
        print 'Adjusting Time Offset'
        first_run = False
    if i % 50 == 0 and data.header.stamp.to_sec() > 1.0:
        sim_time.append(data.header.stamp.to_sec())
        wall_time.append(data.mass)
        cur_time.append(time.Time.now().to_sec() - time_adjust)
        # itrs.append(data.sim_step)
        itrs.append(data.header.seq)
    i += 1


rospy.init_node('time_dilation_inspector')
sub = rospy.Subscriber('/chai/env/Torus/State', ObjectState, msg_cb, queue_size=10)


while not rospy.is_shutdown():
    if len(itrs) > 0:
        if wall_time[-1] >= 0.1 and wall_time[-1] <= 20.0:
            wt_axes, = plt.plot(itrs, wall_time)
            st_axes, = plt.plot(itrs, sim_time)
            ct_axes, = plt.plot(itrs, cur_time)
            plt.grid(True)
            plt.xlabel('No. Iterations')
            plt.ylabel('Time')
            plt.setp(wt_axes, color='r', linewidth=4.0)
            plt.setp(st_axes, color='g', linewidth=1.0, marker='o', markersize=2)
            plt.setp(ct_axes, color='b', linewidth=1.0, marker='x', markersize=2)
            plt.legend([wt_axes, st_axes, ct_axes], ['Chai Wall Time', 'Sim Time', 'Current Time'])
            plt.draw()
            plt.pause(0.001)
plt.show()

