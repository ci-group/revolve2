import os
import pickle
import string
from abc import abstractmethod, ABC

from pyrevolve.experiment.experiment_manager import ExperimentManager


class Memento(ABC):

    experiment_manager = ExperimentManager()

    def __init__(self):
        self.path: string = self.experiment_manager.objects_path
        self.population_memento_path = os.path.join(self.path, "population_management.pickle")

    @abstractmethod
    def load(self, path: string = ""):
        pass

    @abstractmethod
    def export(self, path: string = ""):
        pass

    def dump(self, path: string, export_object):
        with open(path, "wb") as file:
            pickle.dump(export_object, file, pickle.HIGHEST_PROTOCOL)

    def restore(self, path: string) -> object:
        if os.path.exists(path):
            with open(path, "rb") as file:
                return pickle.load(file)