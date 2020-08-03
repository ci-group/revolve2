import string

from nca.experiment.folder_manager import FolderManager
from nca.experiment.abstract.singleton import Singleton


class ExperimentManager(FolderManager, metaclass=Singleton):

    def __init__(self, experiment_name: string = "test"):
        FolderManager.__init__(self, experiment_name)

    def manage(self):
        pass
