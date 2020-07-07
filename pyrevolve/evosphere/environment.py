import os
import string

from pyrevolve.experiment.experiment_manager import ExperimentManager
from pyrevolve.population_ecology.population import Population


class Environment:

    def __init__(self, population: Population, filename: string):
        self.population: Population = population
        self.path: string = os.path.join(ExperimentManager.Instance().world_path, filename)


