from configparser import ConfigParser
import string
import os

from nca.experiment.experiment_manager import ExperimentManager


class Configuration:

    experiment_manager = ExperimentManager()

    def __init__(self, file: string):
        self.path: string = os.path.join(self.experiment_manager.configuration_path, file)
        self.config_parser: ConfigParser = ConfigParser()

        self.config = None

        self.read()

    def config(self):
        return self.config

    def read(self):
        self.config = self.config_parser.read(self.path)
