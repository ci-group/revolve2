from abc import abstractmethod, ABC

from nca.core.abstract.configurations import InitializationConfiguration


class Initialization(ABC):

    def __init__(self):
        self.configuration = InitializationConfiguration()
        pass

    @abstractmethod
    def __call__(self, size: int):
        pass


class SymbolicInitialization(Initialization, ABC):
    pass


class ValuedInitialization(Initialization, ABC):
    pass


class BrainInitialization(Initialization):
    def __call__(self, size: int):
        return []


class MorphologyInitialization(Initialization):
    def __call__(self, size: int):
        return []
