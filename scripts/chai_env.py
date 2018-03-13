#!/usr/bin/env python
from chai_client import ChaiClient
from gym import spaces
import numpy as np
import math
import time
from chai_world import World
from chai_object import Object
from numpy import linalg as LA


class Observation:
    def __init__(self):
        self.state = []
        self.reward = 0.0
        self.prev_reward = 0.0
        self.cur_reward = 0.0
        self.is_done = False
        self.info = {}
        self.sim_step_no = 0

    def cur_observation(self):
        return np.array(self.state), self.reward, self.is_done, self.info


class ChaiEnv:
    def __init__(self):
        self.obj_handle = Object
        self.world_handle = World

        self.chai_client = ChaiClient()
        self.chai_client.create_objs_from_rostopics()
        # self.chai_client.print_summary()
        self.n_skip_steps = 5
        # self.chai_client.start()
        self.enable_step_throttling = True
        self.action = []
        self.obs = Observation()
        self.action_lims = np.array([30, 30, 30, 2, 2, 2])
        self.action_space = spaces.Box(-self.action_lims, self.action_lims)
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(6,))

        self.Base = self.chai_client.get_obj_handle('Base')
        self.prev_sim_step = 0

        pass

    def skip_sim_steps(self, num):
        self.n_skip_steps = num
        self.world_handle.set_num_step_skips(num)

    def set_throttling_enable(self, check):
        self.enable_step_throttling = check
        self.world_handle.enable_throttling(check)

    def make(self, a_name):
        self.obj_handle = self.chai_client.get_obj_handle(a_name)
        self.world_handle = self.chai_client.get_obj_handle('World')
        self.world_handle.enable_throttling(self.enable_step_throttling)
        self.world_handle.set_num_step_skips(self.n_skip_steps)
        if self.obj_handle is None or self.world_handle is None:
            raise Exception

    def reset(self):
        action = [0.0,
                  0.0,
                  0.0,
                  0.0,
                  0.0,
                  0.0]
        return self.step(action)[0]

    def step(self, action):
        action[0:3] = np.clip(action[0:3], -self.action_lims[0:3], self.action_lims[0:3])
        action[3:6] = np.clip(action[3:6], -self.action_lims[3:6], self.action_lims[3:6])
        self.action = action
        assert len(action) == 6
        self.obj_handle.command(action[0],
                                  action[1],
                                  action[2],
                                  action[3],
                                  action[4],
                                  action[5])
        self.world_handle.update()
        self._update_observation()
        return self.obs.cur_observation()

    def render(self, mode):
        print ' I am a {} POTATO'.format(mode)

    def _update_observation(self):
        if self.enable_step_throttling:
            step_jump = 0
            while step_jump < self.n_skip_steps:
                step_jump = self.obj_handle.get_sim_step() - self.prev_sim_step
                time.sleep(0.00001)
            self.prev_sim_step = self.obj_handle.get_sim_step()
            if step_jump > self.n_skip_steps:
                print 'WARN: Jumped {} steps, Default skip limit {} Steps'.format(step_jump, self.n_skip_steps)

        state = self.obj_handle.get_pose()
        self.obs.state = state
        self.obs.reward = self._calculate_reward(state)
        self.obs.is_done = self._check_if_done()
        self.obs.info = self._update_info()

    def _calculate_reward(self, state):
        max_reward = 1
        self.obs.prev_reward = self.obs.cur_reward
        self.obs.cur_reward = max_reward / LA.norm(np.subtract(self.Base.get_pose()[0:3], self.obs.state[0:3]))
        action_penalty = np.sum(np.square(self.action))
        if self.obs.cur_reward > self.obs.prev_reward:
            reward = self.obs.cur_reward
        elif self.obs.cur_reward < self.obs.prev_reward:
            reward = 3 * (self.obs.cur_reward - self.obs.prev_reward)
        else:
            reward = 0.0

        reward = reward - action_penalty
        return reward

    def _check_if_done(self):
        return False

    def _update_info(self):
        return {}
