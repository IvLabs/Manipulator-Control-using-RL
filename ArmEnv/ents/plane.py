import pybullet as p

class Plane:
    def __init__(self, client):
        f_name = 'ArmEnv/resources/simpleplane.urdf'
        p.loadURDF(fileName=f_name,basePosition=[0,0,0],physicsClientId=client)
        