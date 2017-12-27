from chai_client import ChaiClient, Object
from gym import spaces
import numpy as np
import time


class Observation:
    def __init__(self):
        self.state = []
        self.reward = []
        self.is_done = False
        self.info = {'Empty': 'Empty'}

    def cur_observation(self):
        return self.state, self.reward, self.is_done, self.info


class ChaiEnv():
    def __init__(self):
        self.m_obj_handle = []

        self.m_chai_client = ChaiClient()
        self.m_chai_client.create_objs_from_rostopics()
        self.m_chai_client.print_summary()
        self.m_chai_client.start()

        self.m_obs = Observation()
        self.action_space = spaces.Box(-50, 50, shape=(1,6))
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(1, 6))
        pass

    def make(self, a_name):
        self.m_obj_handle = self.m_chai_client.get_obj_handle(a_name)
        if self.m_obj_handle is None:
            raise Exception

    def reset(self):
        action = [[0.0,0.0,0.0,0.0,0.0,0.0]]
        self.step(action)
        self.update_observation()
        return self.m_obs.state

    def step(self, action):
        self.m_obj_handle.command(action[0][0],
                                  action[0][1],
                                  action[0][2],
                                  action[0][3],
                                  action[0][4],
                                  action[0][5])
        time.sleep(0.001)
        self.update_observation()
        return self.m_obs.cur_observation()

    def update_observation(self):
        self.m_obs.state = [self.m_obj_handle.get_pose()]
        self.m_obs.reward = self.calculate_reward()
        self.m_obs.is_done = self.check_if_done()
        self.m_obs.info = self.update_info()

    def calculate_reward(self):
        reward = 0.0
        return reward

    def check_if_done(self):
        return False

    def update_info(self):
        return {'Empty': 'Empty'}
