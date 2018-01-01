#!/usr/bin/env python
import rospy
from chai_msg.msg import ObjectState
import matplotlib.pyplot as plt
import rospy.rostime as time


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

        self.x_axis_type = 0    # 0:'Message Num' | 1:'Sim Step Num'    | 2:'Callback Num'
        self.load_type = 0      # 0:'No Load'     | 1:'Haptic Dev Load' | 2:'Client Load' | 3:'Haptic Dev & Client Load'
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
                if self.counter % 50 == 0:
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
                self.counter += 1

    def run(self):
        rospy.init_node('time_dilation_inspector')
        sub = rospy.Subscriber('/chai/env/Torus/State', ObjectState, self.obj_state_cb, queue_size=10)

        print 'X Axis = ', self.x_axis_dict[self.x_axis_type][0]
        x_axis_indx = self.x_axis_dict[self.x_axis_type][1]

        title_str = self.load_dict[self.load_type] + \
                    ' + ' + self.x_axis_dict[self.x_axis_type][0] + \
                    ' + ' + self.dt_dict[self.dt_type]
        plt.title(title_str)
        while not rospy.is_shutdown():
            if len(x_axis_indx) > 0:
                if self.chai_wall_time[-1] <= self.time_window_lims[1]:
                    ct_axes, = plt.plot(x_axis_indx, self.cur_wall_time)
                    wt_axes, = plt.plot(x_axis_indx, self.chai_wall_time)
                    st_axes, = plt.plot(x_axis_indx, self.chai_sim_time)
                    plt.grid(True)
                    plt.xlabel(self.x_axis_dict[self.x_axis_type][0])
                    plt.ylabel('(Time)')
                    plt.setp(ct_axes, color='b', linewidth=1.0, marker='o', markersize=8)
                    plt.setp(wt_axes, color='r', linewidth=1.0, marker='o', markersize=5)
                    plt.setp(st_axes, color='g', linewidth=1.0, marker='o', markersize=2.5)
                    plt.legend([ct_axes, wt_axes, st_axes], ['Current Time', 'Chai Wall Time', 'Simulation Time'])
                    plt.draw()
                    plt.pause(0.001)
                if self.chai_wall_time[-1] > self.time_window_lims[1]:
                    self.done = True
                    break
        plt.savefig(title_str + '.png', bbox_inches='tight')
        plt.show()


tdObj = TimeDilationAnalysis()
tdObj.run()
