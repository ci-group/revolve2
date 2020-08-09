import os
import string
from typing import List

from nca.core.agent.agents import Agents
from nca.experiment_manager import ExperimentManager


class Environment:

    experiment_manager = ExperimentManager()

    def __init__(self, filename: string, agents_list: List[Agents] = None):
        self.agents_list: List[Agents] = agents_list
        self.path: string = os.path.join(self.experiment_manager.world_path, filename)
