import string
import os


class FolderManager:

    project_path: string = os.path.dirname(os.path.abspath(__file__))

    def __init__(self, experiment_name: string = "test"):
        self.resources_path     = os.path.join(self.project_path,       "resources")
        self.configuration_path = os.path.join(self.resources_path,     "configuration")
        self.world_path         = os.path.join(self.resources_path,     "world")

        self.experiment_path    = self.create_folder(experiment_name)

        self.log_path           = os.path.join(self.experiment_path,    "log")
        self.objects_path       = os.path.join(self.log_path,           "objects")
        self.results_path       = os.path.join(self.log_path,           "results")

    def create_folder(self, experiment_name):
        experiment_path = os.path.join(self.project_path, experiment_name)
        if os.path.exists(self.experiment_path):
            os.mkdir(self.experiment_path)

        return experiment_path
