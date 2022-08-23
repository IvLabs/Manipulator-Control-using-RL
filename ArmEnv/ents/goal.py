import pybullet as p

class Goal:
    def __init__(self, client,position):
        f_name = 'ArmEnv/resources/transparentgoal.urdf'
        p.loadURDF(fileName=f_name,basePosition=position,physicsClientId=client)
        