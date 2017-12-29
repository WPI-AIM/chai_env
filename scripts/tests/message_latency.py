import rospy
from chai_msg.msg import ObjectState
import matplotlib.pyplot as plt
import rospy.rostime as time
import numpy as np


class MessageLatency:
    def __init__(self):
        self.num_readings = 120000
        self.chai_wall_time = np.zeros(self.num_readings)
        self.cur_wall_time = np.zeros(self.num_readings)
        self.counter = 0
        self.first_run = True
        self.msg_itrs = np.zeros(self.num_readings)
        self.sim_itrs = np.zeros(self.num_readings)
        self.latency = np.zeros(self.num_readings)
        self.time_offset = 0
        self.latency_plt = plt
        self.mean_latency = 0.0
        self.time_window = [1.0, 121.0]
        self.done = False

        self.x_axis_str = '(Message Num)'
        self.dt_str = '(Dynamic Time)'
        self.load_str = '(No Load)'
        pass

    def obj_state_cb(self, data):
        if not self.done:
            chai_sim_wall_time = data.wall_time
            process_wall_time = rospy.Time.now().to_sec()
            if chai_sim_wall_time > self.time_window[0]:
                if self.first_run:
                    self.time_offset = process_wall_time - chai_sim_wall_time
                    print 'Adjusting Time Offset: ', self.time_offset
                    self.first_run = False
                else:
                    cur_wall_time_adjusted = process_wall_time - self.time_offset

                    self.chai_wall_time[self.counter] = chai_sim_wall_time
                    self.cur_wall_time[self.counter] = cur_wall_time_adjusted
                    self.latency[self.counter] = cur_wall_time_adjusted - chai_sim_wall_time

                    self.sim_itrs[self.counter] = data.sim_step
                    self.msg_itrs[self.counter] = data.header.seq
                    self.counter += 1

    def compute_mean_latency(self):
        self.mean_latency = self.latency[0:self.counter-1].sum() / self.latency[0:self.counter-1].size
        print 'Mean Latency: ', self.mean_latency, ' | Itrs = ',\
            self.latency[0:self.counter-1].size, ' | Counter = ', self.counter

    def run(self):
        rospy.init_node('message_latency_inspector')
        sub = rospy.Subscriber('/chai/env/Torus/State', ObjectState, self.obj_state_cb, queue_size=1)

        if self.x_axis_str == '(Message Num)':
            itrs = self.msg_itrs
            print 'X Axis = Message Num'
        elif self.x_axis_str == '(Sim Step Num)':
            itrs = self.sim_itrs
            print 'X Axis = Simulation Step Num'

        while not rospy.is_shutdown() and not self.done:
            if len(itrs) > 0:
                if self.chai_wall_time[self.counter-1] > self.time_window[1]:
                    title_str = self.load_str + '+' + self.x_axis_str + '+' + self.dt_str
                    self.done = True

        self.compute_mean_latency()
        self.latency_plt.figure(1)
        self.latency_plt.hist(self.latency[0:self.counter-1], bins='auto', stacked=True)
        self.latency_plt.grid(True)
        self.latency_plt.title(title_str)
        self.latency_plt.show()

        plt.figure(2)
        ct, = plt.plot(itrs[0:self.counter-1], self.cur_wall_time[0:self.counter-1], color='r', linewidth=4.0)
        wt, = plt.plot(itrs[0:self.counter-1], self.chai_wall_time[0:self.counter-1], color='g')
        plt.grid(True)
        plt.legend([ct, wt],['Process Wall Time', 'Chai Wall Time'])
        plt.show()


mlObj = MessageLatency()
mlObj.run()
