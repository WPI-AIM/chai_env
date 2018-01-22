#!/usr/bin/env python
import rospy
from chai_msgs.msg import WorldState
import matplotlib.pyplot as plt
import rospy.rostime as time
import datetime

class TimeDilationAnalysis:
    def __init__(self):
        self.chai_sim_time = []
        self.chai_wall_time = []
        self.cur_wall_time = []
        self.time_window_lims = [0.0, 20.0]
        self.counter = 0
        self.first_run = True
        self.done = False
        self.msg_counter_num = []
        self.simstep_counter_num = []
        self.cb_counter_num = []
        self.initial_time_offset = 0
        self.dynamic_loop_freq = []

        self.x_axis_type = 1    # 0:'Message Num' | 1:'Sim Step Num'    | 2:'Callback Num'
        self.load_type = None   # 0:'No Load'     | 1:'Haptic Dev Load' | 2:'Client Load' | 3:'Haptic Dev & Client Load'
        self.dt_type = 0        # 0:'Dynamic dt'  | 1:'Fixed dt = 0.0005'

        self.x_axis_dict = {0: ['Message Num', self.msg_counter_num],
                            1: ['Sim Step Num', self.simstep_counter_num],
                            2: ['Callback Num', self.cb_counter_num]}
        self.load_dict = {0: 'No Load', 1: 'Haptic Dev Load', 2: 'Client Load', 3: 'Haptic Dev & Client Load'}
        self.dt_dict = {0: 'Dynamic dt', 1: 'Fixed dt = 0.0005'}
        pass

    def capture_window_times(self, time):
            self.time_window_lims[0] = time + 1.0
            self.time_window_lims[1] += self.time_window_lims[0]
            print 'Capturing Time from {} to {}'.format(self.time_window_lims[0], self.time_window_lims[1])

    def obj_state_cb(self, data):
        if not self.done:
            chai_sim_time = data.chai_sim_time
            chai_wall_time = data.chai_wall_time
            if chai_wall_time > self.time_window_lims[0]:
                if self.counter % 100 == 0:
                    if self.first_run:
                        self.capture_window_times(chai_wall_time)
                        self.initial_time_offset = rospy.Time.now().to_sec() - chai_wall_time
                        print 'Adjusting Time Offset'
                        self.first_run = False
                    self.chai_sim_time.append(chai_sim_time)
                    self.chai_wall_time.append(chai_wall_time)
                    self.cur_wall_time.append(time.Time.now().to_sec() - self.initial_time_offset)
                    self.simstep_counter_num.append(data.sim_step)
                    self.msg_counter_num.append(data.header.seq)
                    self.dynamic_loop_freq.append(data.dynamic_loop_freq)
                    if data.n_devices > 0:
                        self.load_type = 1
                    else:
                        self.load_type = 0
                self.counter += 1

    def run(self):
        rospy.init_node('time_dilation_inspector')
        sub = rospy.Subscriber('/chai/env/World/State', WorldState, self.obj_state_cb, queue_size=10)

        print 'X Axis = ', self.x_axis_dict[self.x_axis_type][0]
        x_axis_indx = self.x_axis_dict[self.x_axis_type][1]

        plt.figure(1)
        ax1 = plt.subplot(211)
        ax2 = plt.subplot(212)
        while not rospy.is_shutdown() and not self.done:
            if len(x_axis_indx) > 0:
                if self.chai_wall_time[-1] > self.time_window_lims[1]:
                    self.done = True

                if self.chai_wall_time[-1] <= self.time_window_lims[1]:
                    ax1.cla()
                    ct_axes, = ax1.plot(x_axis_indx, self.cur_wall_time)
                    wt_axes, = ax1.plot(x_axis_indx, self.chai_wall_time)
                    st_axes, = ax1.plot(x_axis_indx, self.chai_sim_time)
                    ax1.grid(True)
                    ax1.set_xlabel(self.x_axis_dict[self.x_axis_type][0])
                    ax1.set_ylabel('(Time)')
                    ax1.legend([ct_axes, wt_axes, st_axes], ['Current Time', 'Chai Wall Time', 'Simulation Time'])

                    ax2.cla()
                    dl_axes, = ax2.plot(x_axis_indx, self.dynamic_loop_freq)
                    ax2.grid(True)
                    ax2.set_xlabel(self.x_axis_dict[self.x_axis_type][0])
                    ax2.set_ylabel('(Dynamic Loop Frequency)')

                    plt.setp(ct_axes, color='b', linewidth=1.0, marker='o', markersize=8)
                    plt.setp(wt_axes, color='r', linewidth=1.0, marker='o', markersize=5)
                    plt.setp(st_axes, color='g', linewidth=1.0, marker='o', markersize=2.5)
                    plt.setp(dl_axes, color='r')
                    plt.draw()
                    plt.pause(0.001)

        title_str = 'Time Dilation: '+ self.load_dict[self.load_type] + \
                    ' + ' + self.x_axis_dict[self.x_axis_type][0] + \
                    ' + ' + self.dt_dict[self.dt_type]
        print title_str
        ax1.set_title(title_str)
        file_str = 'Time Dilation: ' + self.load_dict[self.load_type] + \
                    ' + ' + self.x_axis_dict[self.x_axis_type][0] + \
                    ' + ' + self.dt_dict[self.dt_type] + ': ' + datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.save_graph(plt, file_str)
        plt.show()

    def save_graph(self, handle, str):
            handle.tight_layout()
            handle.savefig('./graphs/' + str + '.eps', bbox_inches='tight', format='eps', dpi=600)


tdObj = TimeDilationAnalysis()
tdObj.run()
