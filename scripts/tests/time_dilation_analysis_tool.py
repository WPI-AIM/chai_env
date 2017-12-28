import rospy
from chai_msg.msg import ObjectState
import matplotlib.pyplot as plt
import rospy.rostime as time


class TimeDilationAnalysis:
    def __init__(self):
        self.sim_time = []
        self.wall_time = []
        self.cur_time = []
        self.itrs = []
        self.counter = 0
        self.first_run = True
        self.msg_itrs = []
        self.sim_itrs = []
        self.time_adjust = 0
        self.plt = plt

        self.x_axis_str = '(Message Num)'
        self.dt_str = '(Fixed Time)'
        self.load_str = '(Haptic Dev Load)'
        pass

    def obj_state_cb(self, data):
        chai_sim_wall_time = data.header.stamp.to_sec()
        if chai_sim_wall_time > 0.0:
            if self.counter % 50 == 0:
                if self.first_run:
                    self.time_adjust = rospy.Time.now().to_sec() - chai_sim_wall_time
                    print 'Adjusting Time Offset'
                    self.first_run = False
                self.sim_time.append(chai_sim_wall_time)
                self.wall_time.append(data.mass)
                self.cur_time.append(time.Time.now().to_sec() - self.time_adjust)
                self.sim_itrs.append(data.sim_step)
                self.msg_itrs.append(data.header.seq)
            self.counter += 1

    def run(self):
        rospy.init_node('time_dilation_inspector')
        sub = rospy.Subscriber('/chai/env/Torus/State', ObjectState, self.obj_state_cb, queue_size=10)

        if self.x_axis_str == '(Message Num)':
            itrs = self.msg_itrs
            print 'X Axix = Message Num'
        elif self.x_axis_str == '(Sim Step Num)':
            itrs = self.sim_itrs
            print 'X Axix = Simulation Step Num'

        plt.ylim([0, 22])
        while not rospy.is_shutdown():
            if len(itrs) > 0:
                if 0.1 < self.wall_time[-1] <= 20.0:
                    ct_axes, = plt.plot(itrs, self.cur_time)
                    wt_axes, = plt.plot(itrs, self.wall_time)
                    st_axes, = plt.plot(itrs, self.sim_time)
                    self.plt.grid(True)
                    self.plt.xlabel(self.x_axis_str)
                    self.plt.ylabel('(Time)')
                    self.plt.setp(wt_axes, color='r', linewidth=4.0)
                    self.plt.setp(st_axes, color='g', linewidth=1.0, marker='|', markersize=10)
                    self.plt.setp(ct_axes, color='b', linewidth=1.0, marker='x', markersize=10)
                    self.plt.legend([wt_axes, st_axes, ct_axes], ['Chai Wall Time', 'Simulation Time', 'Current Time'])
                    self.plt.draw()
                    title_str = self.load_str + '+' + self.x_axis_str + '+' + self.dt_str
                    self.plt.title(title_str)
                    self.plt.pause(0.001)

        plt.savefig(title_str + '.png', bbox_inches='tight')
        plt.show()


tdObj = TimeDilationAnalysis()
tdObj.run()
