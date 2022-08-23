import pybullet as p
import numpy as np


class Arm:
    def __init__(self, client):
        self.client = client

        f_name = 'ArmEnv/resources/kr210.urdf'

        self.arm = p.loadURDF(fileName=f_name, basePosition=[0, 0, 0.005], useFixedBase=1, physicsClientId=client)
        # p.setJointMotorControlArray(self.arm, [2,4,5],p.POSITION_CONTROL,targetPositions = [-1.2,-1.5,-1.5])
        """ for i in range(100):
            p.stepSimulation()

        self.arm_joints = [1,2,3,4] #5 should be 90deg turned
        self.gripper_joints = [9,11,12,14] """

        self.joints = [i for i in range(p.getNumJoints(self.arm))]

    def get_ids(self):
        return self.client, self.id

    def apply_action(self, action, mode, torque_sens, vel_sens):
        ## CHANGE
        ## Make mode changeable

        if (mode == 'T'):
            mode = p.TORQUE_CONTROL
            action = action * torque_sens
            p.setJointMotorControlArray(self.arm, self.joints, mode, forces=action, physicsClientId=self.client)
        elif (mode == 'V'):
            mode = p.VELOCITY_CONTROL
            action = action * vel_sens
            p.setJointMotorControlArray(self.arm, self.joints, mode, targetVelocities=action,
                                        physicsClientId=self.client)

        # p.setJointMotorControlArray(self.arm,self.joints,mode,forces = action,physicsClientId = self.client)

    def get_observation(self):
        ## CHANGE

        obs = []
        self.goal = [0.3, -0.5, 1.1]
        # self.goal = [0.4,0.4,1.1]
        for i in self.joints:
            pos, vel, _, _ = p.getJointState(self.arm, i, physicsClientId=self.client)
            obs.append(pos)
            # obs.append(vel)
        # loc6 = np.array(p.getLinkState(self.arm,6)[0])
        # vel6 = np.array(p.getLinkState(self.arm,6,computeLinkVelocity=1)[6])

        # for i in range(3):
        #    rel = self.goal[i]-loc6[i]
        #    obs.append(rel)
        # for i in range(3):
        #    obs.append(vel6[i])
        for i in range(6):
            obs.append(0)

        # print('loc6: ',loc6)

        return obs