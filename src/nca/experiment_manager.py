import string

from abstract.creational.singleton import Singleton
from nca.folder_manager import FolderManager


class ExperimentManager(metaclass=Singleton):

    def __init__(self, experiment_name: string = "test"):
        self.folders = FolderManager(experiment_name)

    def manage(self):
        pass
