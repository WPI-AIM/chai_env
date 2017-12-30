import rospy
from chai_msg.msg import ObjectState
import matplotlib.pyplot as plt
import rospy.rostime as time


class MessageLatency:
    def __init__(self):
        self.chai_process_wall_time = []    #Array of times for C++ chai process communicated by RosCom
        self.cur_process_wall_time = []     #Array of times for current process
        self.cb_counter = 0
        self.is_first_run = True
        self.msg_counter_num = []
        self.simstep_counter_num = []
        self.latency = []
        self.initial_time_offset = 0
        self.mean_latency = 0.0
        self.time_window_lims = [1.0, 21.0]
        self.done = False

        self.x_axis_type = 0
        self.load_type = 0
        self.dt_type = 1

        self.prev_time = 0

        self.x_axis_dict = {0: ['(Message Num)', self.msg_counter_num], 1: ['(Sim Step Num)', self.simstep_counter_num]}
        self.dt_dict = {0: 'Fixed dt = 0.0005', 1: 'Dynamic dt'}
        self.load_dict = {0: '(No Load)', 1: '(Haptic Dev Load)'}
        pass

    def obj_state_cb(self, data):
        if not self.done:
            if self.prev_time > data.wall_time:
                print 'WARNING', self.prev_time, ' >= ', data.wall_time
            process_wall_time = rospy.Time.now().to_sec()
            if data.wall_time > self.time_window_lims[0]:
                if self.is_first_run:
                    self.initial_time_offset = process_wall_time - data.wall_time
                    print 'Adjusting Time Offset'
                    print 'Sim Wall Time: ', data.wall_time
                    print 'Cur Wall Time: ', process_wall_time
                    print 'Time Offset : ', self.initial_time_offset
                    self.is_first_run = False
                else:
                    cur_wall_time_adjusted = process_wall_time - self.initial_time_offset
                    self.chai_process_wall_time.append(data.wall_time)
                    self.cur_process_wall_time.append(cur_wall_time_adjusted)
                    self.latency.append(cur_wall_time_adjusted - data.wall_time)
                    self.simstep_counter_num.append(data.sim_step)
                    self.msg_counter_num.append(data.header.seq)
                self.cb_counter += 1
        self.prev_time = data.wall_time

    def compute_mean_latency(self):
        self.mean_latency = sum(self.latency) / len(self.latency)
        print 'Mean Latency= ', self.mean_latency, ' | Itrs= ', len(self.latency), ' | Counter=', self.cb_counter

    def run(self):
        rospy.init_node('message_latency_inspector')
        sub = rospy.Subscriber('/chai/env/Torus/State', ObjectState, self.obj_state_cb, queue_size=100)

        print 'X Axis = ', self.x_axis_dict[self.x_axis_type][0]
        itrs = self.x_axis_dict[self.x_axis_type][1]

        while not rospy.is_shutdown() and not self.done:
            if len(itrs) > 0:
                if self.chai_process_wall_time[-1] > self.time_window_lims[1]:
                    title_str = self.load_dict[self.load_type] +\
                                '+' + self.x_axis_dict[self.x_axis_type][0] +\
                                '+' + self.dt_dict[self.dt_type]
                    self.done = True

        self.compute_mean_latency()
        plt.figure(1)
        plt.subplot(311)
        plt.hist(self.latency, bins='auto', stacked=True)
        plt.grid(True)
        plt.title(title_str)

        # plt.figure(2)
        plt.subplot(312)
        lt, = plt.plot(itrs, self.latency, color='r', linewidth=1.0)
        plt.grid(True)
        plt.legend([lt], ['Latency over time'])

        # plt.figure(3)
        plt.subplot(313)
        ct, = plt.plot(itrs, self.cur_process_wall_time, color='r', linewidth=4.0)
        wt, = plt.plot(itrs, self.chai_process_wall_time, color='g')
        plt.grid(True)
        plt.legend([ct, wt], ['Process Wall Time', 'Chai Wall Time'])
        plt.show()


mlObj = MessageLatency()
mlObj.run()
