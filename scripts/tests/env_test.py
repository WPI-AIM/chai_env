from chai_env import ChaiEnv
import time


env = ChaiEnv()
action = env.action_space
env.make('Torus')
env.reset()
for i in range(1,500):
    state, r, d, dict = env.step(env.action_space.sample())
    print r
input = raw_input('Enter input')
