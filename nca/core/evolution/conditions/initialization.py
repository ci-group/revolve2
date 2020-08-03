from abc import abstractmethod

from nca.experiment.configurations import InitializationConfiguration


class Initialization:

    def __init__(self):
        self.configuration = InitializationConfiguration()
        pass

    @abstractmethod
    def algorithm(self, size: int):
        pass

