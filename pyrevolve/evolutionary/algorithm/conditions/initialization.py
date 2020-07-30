from abc import ABC, abstractmethod

from pyrevolve.shared.configurations import InitializationConfiguration


class Initialization(ABC):

    def __init__(self):
        self.configuration = InitializationConfiguration()
        pass

    @abstractmethod
    def algorithm(self, size: int):
        pass

