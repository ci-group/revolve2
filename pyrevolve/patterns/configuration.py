from configparser import ConfigParser
import string
import os

from pyrevolve.experiment.experiment_manager import ExperimentManager


class Configuration:

    def __init__(self, file: string):
        self.path: string = os.path.join(ExperimentManager.Instance(), file)
        self.config_parser: ConfigParser = ConfigParser()

        self.config = None

        self.read()

    def config(self):
        return self.config

    def read(self):
        self.config = self.config_parser.read(self.path)


class ExperimentConfiguration(Configuration):

    def __init__(self):
        super().__init__("experiment.config")


class PopulationConfiguration(Configuration):

    def __init__(self):
        super().__init__("population.config")


class SpeciationConfiguration(Configuration):

    def __init__(self):
        super().__init__("speciation.config")


class AgentConfiguration(Configuration):

    def __init__(self):
        super().__init__("agent.config")


class EvoSphereConfiguration(Configuration):

    def __init__(self):
        super().__init__("evosphere.config")
