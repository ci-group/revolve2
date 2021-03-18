import json
import os


class Config(object):
    def __init__(self, file_path):

        assert os.path.exists(file_path), "Incorrect path to a configuration! Please provide another path."

        with open(file_path) as json_file:
            self.config = json.load(json_file)