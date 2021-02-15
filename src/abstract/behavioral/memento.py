import os
import pickle
import string
from abc import abstractmethod, ABC

from nca.experiment_manager import ExperimentManager


class Memento(ABC):

    experiment_manager = ExperimentManager()

    def __init__(self, name: str):
        self.path: string = self.experiment_manager.folders.objects_path
        self.population_memento_path = os.path.join(self.path, name + ".pickle")

    @abstractmethod
    def load(self, path: string = ""):
        pass

    @abstractmethod
    def export(self, path: string = ""):
        pass

    def dump(self, path: string, export_object):
        with open(os.path.join(self.experiment_manager.folders.log_path, path), "wb") as file:
            pickle.dump(export_object, file, pickle.HIGHEST_PROTOCOL)

    def restore(self, path: string) -> object:
        if os.path.exists(path):
            with open(path, "rb") as file:
                return pickle.load(file)