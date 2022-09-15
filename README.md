# Manipulator-Control-using-RL

This project attempts to use Reinforcement Learning to train a model to perform various control task on a manipulator.

<p align="center">
<img src="https://i.imgur.com/ylzs0zA.gif" width="350" height="350" align="Center">
</p>

## Current State of the Project

Training the model to generalise for random target location.

## Tasks

- The first task for the manipulator is to reach a fixed target location.
- The next task is to learn a generalised policy for the arm to reach any random target location.
- Following a trajectory between two locations

## Environment and Model

### Environment

- Developed a gym environment for Reinforcement Learning on manipulator control using [PyBullet](https://pybullet.org/wordpress/) Physics Simulator.
- The arm's URDF file and Physics Simulator are defined under `/ents/arm.py`.
- The environments are defined under `/envs/`. The action space and observation space are defined here, also the `step` function is defined here.

### Environment Types

##### Point-to-Point

- In this environment the agent starts in a fixed state and it is supposed to reach a fixed target location.

##### Point-to-Random-Point

- In this environment the agent starts from a fixed state and it is supposed to reach a target location whose position is decided at the start of every episode.
- The area where the target spawns can be limited.

#### Environment Parameters

- Episode Length
    - 6500 timesteps
- Torque Sensitivity
    - 200
- Velocity Sensitivity
    - 10
- Position Sensitivity
    - 1
- Maximum Force
    - 300
- Arm Base Position
    - 0, 0, 0.005

### Action Space

There are 3 different Action Types:
- Torque Control (Maximum Torque=200)
- Velocity Control (Maximum Velocity = 10)
- Position Control (Maximum Force = 300)

Keeping in mind, the hardware implementation of the project, we have used `Position Control` as the action space.
We also have trained policies for `Torque Control` action space

## Torque Control

### Observation Space

The observation space(dim=13) consists of:

| Index | Observation | Range |
| -------- | -------- | -------- |
| 0-6 | Joint Position (angular)| - $\pi$ to $\pi$ |
| 7-9 | End Effector Relative location wrt target (cartesian)|-10 to 10|
|10-12|Velocity of End Effector (cartesian)|-10 to 10|

### Model

The models used in this project, for `Torque Control` are from [`Stable Baselines3`](https://stable-baselines3.readthedocs.io/en/master/) a set of reliable implementations of reinforcement learning algorithms using PyTorch.
We have use Proximal Policy Optimization (PPO) algorithm to train our manipulator agent.

#### PPO

Model HyperParameters
- Network Architecture:
    - Policy Network
        - [512, 512, 256, 128]
    - Value Function Approximator
        - [512, 512, 256, 128]
- Activation Function
    - Relu
- Learning Rate
    - 0.0001

### Reward Function

#### Euclidean Distance
The reward function is defined as the negative of Euclidean Distance between the target location and end effector.
```python=
reward = -1*np.linalg.norm(np.array(eeloc)-np.array(self.goal))
```

#### Zero-Reward Box

Defined a cube of side 0.1 units around the goal location, where the agent obtains 0 reward.

### Results and Simulation

<p align="center">
<img src="https://i.imgur.com/q5SX2X8.png" width="650" height="450" align="Center">
</p>

<p align="center">
<img src="https://i.imgur.com/ylzs0zA.gif" width="350" height="350" align="Center">
</p>

<p align="center">
<img src="https://i.imgur.com/qY71Yzy.png" width="650" height="450" align="Center">
</p>


## Position Control

### Observation Space

The observation space(dim=17) consists of:

| Index | Observation | Range |
| -------- | -------- | -------- |
| 0-6 | Joint Position (angular)| - $\pi$ to $\pi$ |
| 7-13 | Joint Angular Velocity|-10 to 10|
|10-12|Target Location (cartesian)|-2 to 2|

### Model

The models used in this project are from [`Stable Baselines3`](https://stable-baselines3.readthedocs.io/en/master/) a set of reliable implementations of reinforcement learning algorithms using PyTorch.
We have use Proximal Policy Optimization (PPO) algorithm to train our manipulator agent.

#### PPO

Model HyperParameters
- Network Architecture:
    - Policy Network
        - [512, 512, 256, 128]
    - Value Function Approximator
        - [512, 512, 256, 128]
- Activation Function
    - Relu
- Learning Rate
    - 0.0001

### Reward Function

#### Euclidean Distance
Firstly, The reward function is defined as the negative of Euclidean Distance between the target location and end effector.
```python=
reward1 = -1*np.linalg.norm(np.array(eeloc)-np.array(self.goal))
```

#### Results and Simulation

<p align="center">
<img src="https://i.imgur.com/xhCqCX3.png" width="650" height="450" align="Center">
</p>

<p align="center">
<img src="https://i.imgur.com/LvUnOAL.gif" width="350" height="350" align="Center">
<br>
Rendering has a delay of 0.1 sec after every action for better visualisation
</p>

<p align="center">
<img src="https://i.imgur.com/kfeUxmJ.png" width="650" height="450" align="Center">
</p>

#### Penalty on Angular Velocity

The vibrations observed above after reaching the target location were decreased by penalizing the agent based on the value of the angular velocity of every joint of the arm.
```python=1
for ang_vel in ang_vel_joints:
    if ang_vel>3:
	    reward2 = -1*ed_dis
```
The Euclidean Distance based reward as defined above was untouched.

### Results and Simulation

<p align="center">
<img src="https://i.imgur.com/cNrx5r4.png" width="650" height="450" align="Center">
</p>

<p align="center">
<img src="https://i.imgur.com/DUr0j6W.gif" width="350" height="350" align="Center">
<br>
Rendering has a delay of 0.1 sec after every action for better visualisation
</p>

<p align="center">
<img src="https://i.imgur.com/tvqRjL7.png" width="650" height="450" align="Center">
</p>

### Problems faced while training the manipulator for Position Control
#### 1. Vibrations after reaching target location
- The vibrations that were observed after reaching the specific target location in the first simulation was possibly because there were no restrictions on the angular velocity of any joint of the manipulator. Thus, even after reaching its location the joints had significant angular velocity, and hence the shaky movement of the arm.
- We overcame this issue by defining another reward function along with the Euclidean Distance, which is to penalize the agent, if the angular velocity of any of its joint is greater than a certain threshold (=3). To keep this reward in proportional to the first reward function, the penalty is equal to 

$reward2 = -1*Euclidean Distance$
- As we can observe in the second simulation, the vibrations were reduced considerably, and the manipulator was quite stable after reaching the target location.

##### A Visualisation to the Manipulator's Vibrations
1. Euclidean Distance of end effector with respect to target location for first case

<p align="center">
<img src="https://i.imgur.com/RkOW2bN.png" width="700" height="400" align="Center">
</p>

2. Euclidean Distance of end effector with respect to target location for second case, where the agent is penalized on its angular velocity
<p align="center">
<img src="https://i.imgur.com/l2kmJKe.png" width="700" height="400" align="Center">
</p>