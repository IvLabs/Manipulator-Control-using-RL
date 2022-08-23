import pybullet as p
from time import sleep
import numpy as np

p.connect(p.GUI)
p.setGravity(0,0,-10)
#arm = p.loadURDF( 'univ_arm/resources/kuka_lbr_iiwa_support/urdf/lbr_iiwa_14_r820.urdf',[0,0,0],useFixedBase=1)
arm = p.loadURDF('ArmEnv/resources/model.urdf',[0,0,0],useFixedBase=0)

joints = [i for i in range(p.getNumJoints(arm))]

for i in range(p.getNumJoints(arm)):
    info = p.getJointInfo(arm,i)
    print(info[0:3])

l_bound = -1*np.pi
u_bound =  1*np.pi

j1 = p.addUserDebugParameter('j1',l_bound,u_bound)
j2 = p.addUserDebugParameter('j2',l_bound,u_bound)
j3 = p.addUserDebugParameter('j3',l_bound,u_bound)
j4 = p.addUserDebugParameter('j4',l_bound,u_bound)
j5 = p.addUserDebugParameter('j5',l_bound,u_bound)
j6 = p.addUserDebugParameter('j6',l_bound,u_bound)
j7 = p.addUserDebugParameter('j7',l_bound,u_bound)
#j8 = p.addUserDebugParameter('j8',l_bound,u_bound)

mode = p.POSITION_CONTROL
flag=0
verbose = 1
kray = 0
while(True):


    flag += 1
    if(flag == 10000):
        flag = 1

    v1 = p.readUserDebugParameter(j1)
    v2 = p.readUserDebugParameter(j2)
    v3 = p.readUserDebugParameter(j3)
    v4 = p.readUserDebugParameter(j4)
    v5 = p.readUserDebugParameter(j5)
    v6 = p.readUserDebugParameter(j6)
    v7 = p.readUserDebugParameter(j7)
    #v8 = p.readUserDebugParameter(j8)
    if(flag == 9999 and verbose):
        """ for i in joints:
            pos,vel,_,_ = p.getJointState(arm,i)
            print(i,end='   ')
            print("Pos:",pos,end='   ')
            print("Vel:",vel) """
        #print('LINK STATE:',p.getLinkState(arm,6)[0])
        if(kray == 0):
            print('kray')
            #p.setJointMotorControlArray(arm,[0,1,2,3,4,5,6],p.POSITION_CONTROL,forces = 10.2*np.ones(7),targetPositions = [0,0.13,0,-1,0,0.35,0])
            kray = 1
        elif(kray == 1):
            print('not kray')
            #p.setJointMotorControlArray(arm,[0,1,2,3,4,5,6],p.POSITION_CONTROL,forces = 10.2*np.ones(7),targetPositions = [0,0.4,0,-1.6,0,-0.85,0])
            kray = 0

    #p.setJointMotorControlArray(arm, [0, 1, 2, 3, 4, 5, 6], p.POSITION_CONTROL, targetPositions=[0, 1.5, 0, 0, 0, 0, 0])
    #p.setJointMotorControlArray(arm,[0,1,2,3,4,5,6,9,12],mode,targetVelocities = [0,v1,v2,v3,v4,v5,v6,v7,v8])
    p.setJointMotorControlArray(arm,[0,1,2,3,4,5,6],mode,forces = 60*np.ones(7),targetPositions= [v1,v2,v3,v4,v5,v6,v7])
    p.stepSimulation()