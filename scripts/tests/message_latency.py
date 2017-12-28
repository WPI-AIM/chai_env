import rospy
from chai_msg.msg import ObjectState
import matplotlib.pyplot as plt
import rospy.rostime as time


class MessageLatency:
    def __init__(self):
        self.chai_wall_time = []
        self.cur_wall_time = []
        self.counter = 0
        self.first_run = True
        self.msg_itrs = []
        self.sim_itrs = []
        self.latency = []
        self.time_adjust = 0
        self.latency_plt = plt
        self.mean_latency = 0.0
        self.time_window = [0.1, 20.0]
        self.done = False

        self.x_axis_str = '(Message Num)'
        self.dt_str = '(Fixed Time)'
        self.load_str = '(Haptic Dev Load)'
        pass

    def obj_state_cb(self, data):
        chai_sim_wall_time = data.wall_time
        process_wall_time = rospy.Time.now().to_sec()
        if chai_sim_wall_time > 1.0:
            if self.first_run:
                self.time_adjust = process_wall_time - chai_sim_wall_time
                print 'Adjusting Time Offset'
                self.first_run = False
            else:
                cur_wall_time_adjusted = process_wall_time - self.time_adjust

                self.chai_wall_time.append(chai_sim_wall_time)
                self.cur_wall_time.append(cur_wall_time_adjusted)
                self.latency.append(cur_wall_time_adjusted - chai_sim_wall_time)

                self.sim_itrs.append(data.sim_step)
                self.msg_itrs.append(data.header.seq)
            self.counter += 1

    def compute_mean_latency(self):
        self.mean_latency = sum(self.latency) / len(self.latency)
        print 'Mean Latency is ', self.mean_latency, ' | No of itrs = ', len(self.latency)

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
                if self.chai_wall_time[-1] > self.time_window[1]:
                    title_str = self.load_str + '+' + self.x_axis_str + '+' + self.dt_str
                    self.done = True

        self.compute_mean_latency()
        self.latency_plt.figure(1)
        self.latency_plt.hist(self.latency, bins='auto', stacked=True)
        self.latency_plt.grid(True)
        self.latency_plt.title(title_str)
        self.latency_plt.show()

        plt.figure(2)
        ct, = plt.plot(itrs, self.cur_wall_time, color='r', linewidth=4.0)
        wt, = plt.plot(itrs, self.chai_wall_time, color='g')
        plt.grid(True)
        plt.legend([ct, wt],['Process Wall Time', 'Chai Wall Time'])
        plt.show()


mlObj = MessageLatency()
mlObj.run()
