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

x_axis_str = '(Message Num)'
dt_str = '(Fixed Time)'
load_str = '(Haptic Dev Load)'


def msg_cb(data):
    global i, first_run, time_adjust
    chai_sim_wall_time = data.header.stamp.to_sec()
    if chai_sim_wall_time > 0.0:
        if i % 50 == 0:
            if first_run:
                time_adjust = rospy.Time.now().to_sec() - chai_sim_wall_time
                print 'Adjusting Time Offset'
                first_run = False

            sim_time.append(chai_sim_wall_time)
            wall_time.append(data.mass)
            cur_time.append(time.Time.now().to_sec() - time_adjust)
            # itrs.append(data.sim_step)
            itrs.append(data.header.seq)
        i += 1


rospy.init_node('time_dilation_inspector')
sub = rospy.Subscriber('/chai/env/Torus/State', ObjectState, msg_cb, queue_size=10)

plt.ylim([0,22])
# plt.xlim([0,22000])
title_str = 0
while not rospy.is_shutdown():
    if len(itrs) > 0:
        if wall_time[-1] >= 0.1 and wall_time[-1] <= 20.0:
            ct_axes, = plt.plot(itrs, cur_time)
            wt_axes, = plt.plot(itrs, wall_time)
            st_axes, = plt.plot(itrs, sim_time)
            plt.grid(True)
            plt.xlabel(x_axis_str)
            plt.ylabel('(Time)')
            plt.setp(wt_axes, color='r', linewidth=4.0)
            plt.setp(st_axes, color='g', linewidth=1.0, marker='|', markersize=10)
            plt.setp(ct_axes, color='b', linewidth=1.0, marker='x', markersize=10)
            plt.legend([wt_axes, st_axes, ct_axes], ['Chai Wall Time', 'Simulation Time', 'Current Time'])
            plt.draw()
            title_str = load_str + '+' + x_axis_str + '+' + dt_str
            plt.title(title_str)
            plt.pause(0.001)

plt.savefig(title_str + '.png', bbox_inches='tight')
plt.show()


