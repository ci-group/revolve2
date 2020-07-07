import os
import string
from abc import abstractmethod, ABC
from typing import List

from pyrevolve.experiment.experiment_manager import ExperimentManager


class Memento(ABC):

    def __init__(self):
        self.path: string = ExperimentManager.Instance().objects_path
        self.population_memento_path = os.path.join(self.path, "population_management.pickle")

    @abstractmethod
    def load(self):
        pass

    @abstractmethod
    def recover(self):
        pass
