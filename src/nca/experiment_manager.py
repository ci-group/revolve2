import string

from nca.core.abstract.singleton import Singleton
from nca.folder_manager import FolderManager


class ExperimentManager(FolderManager, metaclass=Singleton):

    def __init__(self, experiment_name: string = "test"):
        FolderManager.__init__(self, experiment_name)

    def manage(self):
        pass
