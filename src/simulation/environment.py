import os
import string

import gym as gym

from nca.experiment_manager import ExperimentManager
from simulation.simulator.simulator_helper import SimulatorType


class Environment(gym.Env):

    experiment_manager = ExperimentManager()

    def __init__(self, filename: string):
        self.path: string = os.path.join(self.experiment_manager.world_path, filename)
        self.simulator_type = None

        self.observation_space = None
        self.action_space = None
        self.state = None

    def step(self, action):
        pass

    def reset(self):
        pass

    def render(self, mode='human'):
        pass

    def close(self):
        pass


class GazeboEnvironment(Environment):
    def __init__(self, filename: string):
        super().__init__("gazebo/" + filename)
        self.simulator_type = SimulatorType.GAZEBO


class MalmoEnvironment(Environment):
    def __init__(self, filename: string):
        super().__init__("gazebo/" + filename)
        self.simulator_type = SimulatorType.MALMO


class UnityEnvironment(Environment):
    def __init__(self, filename: string):
        super().__init__("gazebo/" + filename)
        self.simulator_type = SimulatorType.Unity
