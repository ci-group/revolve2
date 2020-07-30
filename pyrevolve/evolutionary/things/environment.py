import os
import string
from typing import List

from pyrevolve.evolutionary import Agents
from pyrevolve.experiment.experiment_manager import ExperimentManager
from pyrevolve.evolutionary.ecology.population import Population


class Environment:

    experiment_manager = ExperimentManager()

    def __init__(self, filename: string, agents: List[Agents] = None):
        self.agents: List[Agents] = agents
        self.path: string = os.path.join(self.experiment_manager.world_path, filename)
