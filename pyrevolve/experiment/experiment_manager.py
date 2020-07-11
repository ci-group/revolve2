import string

from pyrevolve.experiment.folder_manager import FolderManager
from pyrevolve.patterns.abstract.singleton import Singleton


class ExperimentManager(Singleton, FolderManager):

    def __init__(self, experiment_name: string = "test"):
        Singleton.__init__(self)
        FolderManager.__init__(self, experiment_name)

    def manage(self):
        pass
