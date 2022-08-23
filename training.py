import gym
import ArmEnv
import matplotlib.pyplot as plt
import numpy as np
import pickle
import torch as th

from stable_baselines3.common.monitor import Monitor
from stable_baselines3 import PPO
from stable_baselines3 import TD3
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.callbacks import CallbackList
from stable_baselines3 import SAC

env = gym.make('PointToPoint-v0',mode='P',P_sens=1,P_max_force=60)
env = Monitor(env,'monitor_2308')

eval_callback = EvalCallback(env, best_model_save_path='./logs/',
                             log_path='./logs/', eval_freq=65000,
                             render=False)
checkpoint_callback = CheckpointCallback(save_freq=97500, save_path='./logs/',
                                         name_prefix='rl_model')
callback = CallbackList([checkpoint_callback, eval_callback])

#model = SAC('MlpPolicy',env,verbose=1,device='cuda')
policy_kwargs = dict(ortho_init=False,activation_fn=th.nn.ReLU,net_arch=[dict(pi=[512,512,256,128], vf=[512,512,265,128])])
#model = PPO.load('logs/rl_model_3835480_steps.zip',env)
model = PPO('MlpPolicy',env,policy_kwargs=policy_kwargs,verbose=1,learning_rate=0.0001)
#model.learn(3250000,callback=callback,reset_num_timesteps=False)
model.learn(1300000,callback=callback,reset_num_timesteps=False)

t = env.get_episode_rewards()
model.save("arm_2308_modeP")
del model


file_name = "rewards_2308.pkl"
op_file = open(file_name,'wb')
pickle.dump(t, op_file)
op_file.close()

fi,a = plt.subplots()
a.plot(np.arange(len(t)),t)
plt.show()
