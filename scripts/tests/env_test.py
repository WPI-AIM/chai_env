#!/usr/bin/env python
from chai_env import ChaiEnv
import time


env = ChaiEnv()
action = env.action_space
env.make('Torus')
time.sleep(1)
env.reset()
total = 5000
for i in range(1,total):
    state, r, d, dict = env.step(env.action_space.sample())
    if i % 50 == 0:
        print 'Reward: ', r, 'Steps: ', i, ' \ ', total
