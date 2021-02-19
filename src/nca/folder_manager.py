import string
import os
import rootpath


class FolderManager:

    project_path: string = rootpath.detect()

    def __init__(self, experiment_name: string):
        self.resources_path = os.path.join(self.project_path, "resources")
        self.log_path = os.path.join(self.project_path, "log")
        self.experiment_path = self.create_folder(self.log_path, experiment_name)

        self.configuration_path = os.path.join(self.resources_path, "configuration")
        self.world_path = os.path.join(self.resources_path, "world")
        self.config_path = os.path.join(self.resources_path, "config")

        self.objects_path = os.path.join(self.experiment_path, "objects")
        self.results_path = os.path.join(self.experiment_path, "results")

    @staticmethod
    def create_folder(path, experiment_name):
        experiment_path = os.path.join(path, experiment_name)
        if os.path.exists(experiment_name):
            os.mkdir(experiment_name)

        return experiment_path
