import gym
import ArmEnv
import matplotlib.pyplot as plt
import numpy as np
import pickle
from collections import deque
from stable_baselines3.common.monitor import Monitor
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3 import SAC
import time



env = gym.make('PointToRandomPoint-v0',gui=True,mode='P',record=True,P_sens=1,P_max_force=300)

model = PPO.load("logs/rl_model_877500_steps.zip", env=env,custom_objects={"learning_rate": 0.0,
            "lr_schedule": lambda _: 0.0,
            "clip_range": lambda _: 0.0,})
#model = PPO.load("trial.zip", env=env)
rew = []
rew1 = []
rew2 = []

eelocx = []
eelocy = []
eelocz = []
ed_list = []

for i in range(1):
    done = False
    obs = env.reset()
    #time.sleep(0)
    #'''
    while not done:
        #time.sleep(0.05)
        action, _state =model.predict(obs)
        #action = np.array([0,-1,0,0,0,0,0])
        obs,reward,done,dic = env.step(action)
        #reward1,reward2 = reward
        #eeloc = dic["eeloc"]
        ed = dic["ed"]
        rew_list = dic["rew_dic"]
        reward1=rew_list[0]
        reward2=rew_list[1]
    #'''



        print(obs[0:8])
        #print(action)
        #print(action)
        if i==0:
            rew1.append(reward1)
            rew2.append(reward2)
            rew.append(reward)
            #eelocx.append(0.5-eeloc[0])
            #eelocy.append(0-eeloc[1])
            #eelocz.append(0.9-eeloc[2])
            ed_list.append(ed)



t = np.arange(len(rew))
#print(sum(rew))


fig,ax = plt.subplots()
ax.plot(t,rew)
ax.set_title("Testing")
ax.set_xlabel("Timesteps")
ax.set_ylabel("Rewards")

fig1, ax1 = plt.subplots()
ax1.plot(t,rew1,label="Euclidian")
ax1.plot(t,rew2,label="Ang_Vel")
ax.set_title("Reward Functions")
ax.set_xlabel("Timesteps")
ax.set_ylabel("Rewards")
plt.legend()

#fig2,ax2 = plt.subplots()
#ax2.plot(t,rew2)
#ax2.set_title("Testing")
#ax2.set_xlabel("Timesteps")
#ax2.set_ylabel("Rewards")

#fig1, ax1 = plt.subplots()
#ax1.plot(t, eelocx)
#ax1.set_title("eelocx")
#ax1.set_xlabel("Timesteps")
#ax1.set_ylabel("eelocx")

#fig2, ax2 = plt.subplots()
#ax2.plot(t, eelocy)
#ax2.set_title("eelocy")
#ax2.set_xlabel("Timesteps")
#ax2.set_ylabel("eelocy")

#fig3, ax3 = plt.subplots()
#ax3.plot(t, eelocz)
#ax3.set_title("eelocz")
#ax3.set_xlabel("Timesteps")
#ax3.set_ylabel("eelocz")

fig4, ax4 = plt.subplots()
ax4.plot(t,ed_list)
ax4.set_title("Euclidean Dis")
ax4.set_xlabel("Timesteps")
ax4.set_ylabel("Distance")

plt.show()
#goal: [0.4,0,1.1]  loc6:  [0.44738302 0.04233105 1.11781087]
#goal: [0.6,0,0.9]  loc6:  [0.5669553  0.07910315 1.04583075]
#goal: [0.3,0,1.2]  loc6:  [0.27454447 0.03467133 1.20632827]