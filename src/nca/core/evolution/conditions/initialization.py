from abc import abstractmethod, ABC

import numpy as np

from nca.core.abstract.configurations import InitializationConfiguration
from nca.core.genome.grammar.symbol import Symbol


class Initialization(ABC):

    def __init__(self, configuration: InitializationConfiguration = InitializationConfiguration()):
        self.configuration = configuration
        pass

    @abstractmethod
    def __call__(self, size: int):
        pass


class SymbolicInitialization(Initialization, ABC):

    def __init__(self, symbol):
        super().__init__()
        if isinstance(symbol, type(Symbol)):
            self.symbol: type(Symbol) = symbol
            self.elements = None
        else:
            self.elements = symbol
            self.symbol: type(Symbol) = symbol.initialization.symbol

    def __call__(self, size: int):
        return [np.random.choice(self.symbol, p=self.symbol.probabilities(self.elements)) for i in range(size)]


class ValuedInitialization(Initialization, ABC):
    pass


class BrainInitialization(Initialization):
    def __call__(self, size: int):
        return []


class MorphologyInitialization(Initialization):
    def __call__(self, size: int):
        return []
