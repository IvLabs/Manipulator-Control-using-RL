# Manipulator-Control-using-DRL
- This project attempts to use Reinforcement Learning to train a model to perform various control task on a manipulator.

<p align="center">
<img src="https://i.imgur.com/ylzs0zA.gif" width="350" height="350" align="Center">
</p>

## Current State of the Project
Training the model to generalise for random target location.

## Task
- The first task for the manipulator is to reach a fixed target location.
- The next task is to learn a generalised policy for the arm to reach any random target location.

## Environment and Model

### Environment
- Developed a gym environment for Reinforcement Learning on manipulator control using [PyBullet](https://pybullet.org/wordpress/) Physics Simulator.
- The arm's URDF file and Physics Simulator are defined under `/ents/arm.py`.
- The environment is defined under `/envs/arm_ptop.py`. The action space and observation space are defined here, also the `step` function is defined here.
#### Action Space
There are 2 different Action Types:
- Torque Control (Maximum Torque=200)
- Velocity Control (Maximum Velocity = 10)
Keeping in mind, the hardware implementation of the project, we have used `Torque Control` as the action space.

#### Observation Space
The observation space(dim=13) consists of:

| Index | Observation | Range |
| -------- | -------- | -------- |
| 0-6 | Joint Position (angular)| -$\pi$ to $\pi$ |
| 7-9 | End Effector Relative location wrt target (cartesian)|-10 to 10|
|10-12|Velocity of End Effector (cartesian)|-10 to 10|

#### Reward Function
The reward function is defined as the negative of Eucledian Distance between the target location and end effector.
```python=
reward = -1*np.linalg.norm(np.array(eeloc)-np.array(self.goal))
```
##### Zero-Reward Box
Defined a cube of side 0.1 units around the goal location, where the agent obtains 0 reward.

#### Environment Parameters
- Episode Length
    - 6500 timesteps
- Torque Sensitivity
    - 200
- Velocity Sensitivity
    - 10
- Arm Base Position
    - 0, 0, 0.005

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
