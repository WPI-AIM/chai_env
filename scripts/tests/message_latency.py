#!/usr/bin/env python
import rospy
from chai_msgs.msg import WorldState
import matplotlib.pyplot as plt
import datetime


class MessageLatency:
    def __init__(self):
        self.chai_process_wall_time = []    #Array of times for C++ chai process communicated by RosCom
        self.cur_process_wall_time = []     #Array of times for current process
        self.cb_counter = 0
        self.is_first_run = True
        self.msg_counter_num = []
        self.simstep_counter_num = []
        self.cb_counter_num = []
        self.latency_list = []
        self.latency_exceptions = 0
        self.initial_time_offset = 0
        self.mean_latency = 0.0
        self.time_window_lims = [0.0, 20.0]
        self.window_times_captured = False
        self.done = False
        self.queue_size = 50
        self.figure_ctr = 0
        self.dt_cur_wall_times = []
        self.dt_chai_wall_times = []

        self.x_axis_type = 0  # 0:'Message Num' | 1:'Sim Step Num'    | 2:'Callback Num'
        self.load_type = None  # 0:'No Load'     | 1:'Haptic Dev Load' | 2:'Client Load' | 3:'Haptic Dev & Client Load'
        self.dt_type = 1  # 0:'Dynamic dt'  | 1:'Fixed dt = 0.0005'

        self.x_axis_dict = {0: ['Message Num', self.msg_counter_num],
                            1: ['Sim Step Num', self.simstep_counter_num],
                            2: ['Callback Num', self.cb_counter_num]}
        self.load_dict = {0: 'No Load', 1: 'Haptic Dev Load', 2: 'Client Load', 3: 'Haptic Dev & Client Load'}
        self.dt_dict = {0: 'Dynamic dt', 1: 'Fixed dt = 0.0005'}
        pass

    def capture_window_times(self, time):
        if not self.window_times_captured:
            self.time_window_lims[0] = time + 1.0
            self.time_window_lims[1] += self.time_window_lims[0]
            print 'Capturing Time from {} to {}'.format(self.time_window_lims[0], self.time_window_lims[1])
            self.window_times_captured = True

    def obj_state_cb(self, data):
        if not self.done:
            chai_sim_wall_time = data.header.stamp.to_sec()
            process_wall_time = rospy.Time.now().to_sec()

            if chai_sim_wall_time > self.time_window_lims[0]:
                if self.is_first_run:
                    self.capture_window_times(data.chai_wall_time)
                    self.initial_time_offset = chai_sim_wall_time - data.chai_wall_time
                    print 'ROS & CHAI Clock Offset in C++ Server: ', self.initial_time_offset
                    print 'CHAI Wall Time after offset          : ', chai_sim_wall_time - self.initial_time_offset
                    print 'Cur Process Wall Time after offset   : ', process_wall_time - self.initial_time_offset
                    self.is_first_run = False

                self.chai_process_wall_time.append(chai_sim_wall_time - self.initial_time_offset)
                self.cur_process_wall_time.append(process_wall_time - self.initial_time_offset)
                self.latency_list.append(process_wall_time - chai_sim_wall_time)

                self.simstep_counter_num.append(data.sim_step)
                self.msg_counter_num.append(data.header.seq)
                self.cb_counter_num.append(self.cb_counter)
                self.cb_counter += 1
                if data.n_devices > 0:
                    self.load_type = 1
                else:
                    self.load_type = 0

    def compute_mean_latency(self):
        self.mean_latency = sum(self.latency_list) / len(self.latency_list)
        print 'Mean Latency= ', self.mean_latency, ' | Itrs= ', len(self.latency_list), ' | Counter=', self.cb_counter

        total_packets = (self.msg_counter_num[-1] + 1) - self.msg_counter_num[0]
        total_packets_rcvd = len(self.msg_counter_num)
        percent_packets_rcvd = (total_packets_rcvd * 1.0) / (total_packets * 1.0)

        print 'Total packets sent by C++ Server: ', total_packets
        print 'Total packets received by Client: ', total_packets_rcvd
        print 'Percentage of packets received  : {}%'.format(100 * percent_packets_rcvd)

    def calculate_packets_dt(self, list):
        new_list = []
        for idx in range(1, len(list)-1):
            new_list.append(list[idx] - list[idx-1])
        return new_list

    def run(self):
        rospy.init_node('message_latency_inspector')
        sub = rospy.Subscriber('/chai/env/World/State', WorldState, self.obj_state_cb, queue_size=self.queue_size)

        print 'X Axis = ', self.x_axis_dict[self.x_axis_type][0]
        x_axis_indx = self.x_axis_dict[self.x_axis_type][1]

        while not rospy.is_shutdown() and not self.done:
            if len(x_axis_indx) > 0:
                if self.chai_process_wall_time[-1] > self.time_window_lims[1]:
                    self.done = True
                    self.compute_mean_latency()
                    self.dt_cur_wall_times = self.calculate_packets_dt(self.cur_process_wall_time)
                    self.dt_chai_wall_times = self.calculate_packets_dt(self.chai_process_wall_time)
                    for keys, item in self.x_axis_dict.iteritems():
                        title_str = self.load_dict[self.load_type] +\
                                    '+' + item[0] +\
                                    '+' + self.dt_dict[self.dt_type]
                        x_axis_indx = item[1]
                        self.generate_graphs(title_str, x_axis_indx, item[0])
                    plt.show()

    def generate_graphs(self, title_str, x_axis_indx, x_label):
        plt1_str = 'Latency Hist: ' + title_str
        self.figure_ctr += 1
        plt.figure(self.figure_ctr)
        plt.subplot(311)
        plt.hist(self.latency_list, bins='auto', stacked=True)
        plt.grid(True)
        plt.xlabel('Latency')
        plt.ylabel('No. Packets')
        plt.title(plt1_str)

        # plt.figure(2)
        plt.subplot(312)
        lt, = plt.plot(x_axis_indx, self.latency_list, color='r', linewidth=1.0)
        plt.grid(True)
        plt.legend([lt], ['Latency over time'])
        plt.xlabel(x_label)
        plt.ylabel('Latency')

        # plt.figure(3)
        plt.subplot(313)
        ct, = plt.plot(x_axis_indx, self.cur_process_wall_time, color='r', linewidth=4.0)
        wt, = plt.plot(x_axis_indx, self.chai_process_wall_time, color='g')
        plt.grid(True)
        plt.legend([ct, wt], ['Process Wall Time', 'Chai Wall Time'])
        plt.xlabel(x_label)
        plt.ylabel('Time Comparison')

        self.save_graph(plt, plt1_str + datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"))

        self.figure_ctr += 1
        plt2_str = 'Recv vs Read: ' + title_str
        plt.figure(self.figure_ctr)
        plt.subplot(311)
        cur_dt_axes_1 = plt.scatter(x_axis_indx[0:-2], self.dt_cur_wall_times, color='r', marker='.', s=5)
        plt.legend([cur_dt_axes_1], ['Cur Process dt'])
        plt.grid(True)
        plt.title(plt2_str)
        plt.xlabel(x_label)
        plt.ylabel('Read Time')
        plt.subplot(312)
        chai_dt_axes_1 = plt.scatter(x_axis_indx[0:-2], self.dt_chai_wall_times, color='g', marker='.', s=5)
        plt.grid(True)
        plt.legend([cur_dt_axes_1], ['Chai Process dt'])
        plt.xlabel(x_label)
        plt.ylabel('Recv Time')
        plt.subplot(313)
        cur_dt_axes_2 = plt.scatter(x_axis_indx[0:-2], self.dt_cur_wall_times, color='r', marker='.', s=5)
        chai_dt_axes_2 = plt.scatter(x_axis_indx[0:-2], self.dt_chai_wall_times, color='g', marker='.', s=5)
        plt.grid(True)
        plt.legend([cur_dt_axes_2, chai_dt_axes_2], ['Cur Process dt', 'Chai Process dt'])
        plt.xlabel(x_label)
        plt.ylabel('Recv vs Read Time')

        self.save_graph(plt, plt2_str + datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"))

    def save_graph(self, handle, str):
        handle.tight_layout()
        handle.savefig('./graphs/' + str + '.eps', bbox_inches='tight', format='eps', dpi=600)

mlObj = MessageLatency()
mlObj.run()
