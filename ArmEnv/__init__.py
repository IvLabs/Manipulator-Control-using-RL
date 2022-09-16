from gym.envs.registration import register

register(
    id = 'PointToPoint-v0',
    entry_point = 'ArmEnv.envs:PointToPoint'
)

register(
    id = 'PointToRandomPoint-v0',
    entry_point = 'ArmEnv.envs:PointToRandomPoint'
)