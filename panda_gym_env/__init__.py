from gym.envs.registration import register

register(
    id='panda_gym_env-v0',
    entry_point='panda_gym_env.envs:PandaGymEnv'
)
