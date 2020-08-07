from abc import abstractmethod

from nca.core.abstract.configurations import InitializationConfiguration


class Initialization:

    def __init__(self):
        self.configuration = InitializationConfiguration()
        pass

    @abstractmethod
    def algorithm(self, size: int):
        pass

