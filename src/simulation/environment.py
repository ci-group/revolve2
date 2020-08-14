import gym


class Environment(gym.Env):

    def __init__(self):
        self.observation_space = None
        self.action_space = None
        self.state = None
        self.measures = None

    def step(self, action):
        # add to measures
        pass

    def reset(self):
        pass

    def render(self, mode='human'):
        pass

    def close(self):
        pass
