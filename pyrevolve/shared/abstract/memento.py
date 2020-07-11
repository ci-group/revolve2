import os
import string
from abc import abstractmethod, ABC

from pyrevolve.experiment.experiment_manager import ExperimentManager


class Memento(ABC):

    experiment_manager = ExperimentManager()

    def __init__(self):
        self.path: string = self.experiment_manager.objects_path
        self.population_memento_path = os.path.join(self.path, "population_management.pickle")

    @abstractmethod
    def load(self):
        pass

    @abstractmethod
    def export(self):
        pass
