
import gym


class GymTemplate(gym.Env):

    def __init__(self):
        self.done = False

    def reset(self):
        self.done = False

        return self.state()

    def step(self, action):

        return self.state()

    def state(self):
        return self._observation(), self._reward(), self._stop_condition(), self._info()

    def render(self, mode='human'):
        pass

    def _observation(self):
        return None

    def _reward(self):
        return None

    def _stop_condition(self):
        self.done = False
        return self.done

    def _info(self):
        return None
