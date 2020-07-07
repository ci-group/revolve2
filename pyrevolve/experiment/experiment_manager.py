from pyrevolve.experiment.folder_manager import FolderManager
from pyrevolve.patterns.abstract.singleton import Singleton
from pyrevolve.patterns.configuration import ExperimentConfiguration


class ExperimentManager(Singleton, FolderManager):

    def __init__(self):
        super(Singleton).__init__()
        super(FolderManager).__init__()
        self.configuration: ExperimentConfiguration = ExperimentConfiguration(self.configuration_path)

    def manage(self):
        pass
