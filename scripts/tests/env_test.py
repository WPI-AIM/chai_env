from chai_env import ChaiEnv



env = ChaiEnv()
action = env.action_space
env.make('Torus')
env.reset()
import time
time.sleep(2)
for i in range(1,500):
    state, r, d, dict = env.step(env.action_space.sample())
    print state
env.reset()
