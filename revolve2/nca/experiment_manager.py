import string

from revolve2.abstract.creational.singleton import Singleton
from revolve2.nca.folder_manager import FolderManager


class ExperimentManager(metaclass=Singleton):

    def __init__(self, experiment_name: string = "test"):
        self.folders = FolderManager(experiment_name)

    def manage(self):
        pass
