import random
from abc import ABC, abstractmethod

import numpy as np

from pyrevolve.evolutionary.algorithm.genome.representation import Representation
from pyrevolve.evolutionary.algorithm.genome.representations.l_system.alphabet import Alphabet


class DirectRepresentation(Representation, ABC):

    def __init__(self, data_type: type):
        super().__init__()
        self.data_type = data_type

        self.init(None)

    def compatibility(self, other):
        return np.linalg.norm(self.genome - other.genome)


class BinaryRepresentation(DirectRepresentation):

    def __init__(self):
        super().__init__(bool)

    def _initialize(self):
        self.genome = np.random.randint(2, size=self.configuration.genome_size)


class IntegerRepresentation(DirectRepresentation):

    def __init__(self):
        super().__init__(int)

    def _initialize(self):
        self.genome = np.random.randint(self.configuration.minimum_value, self.configuration.maximum_value,
                                        size=self.configuration.genome_size)


class RealValuedRepresentation(DirectRepresentation):

    def __init__(self):
        super().__init__(float)

    def _initialize(self):
        self.genome = np.random.uniform(self.configuration.minimum_value, self.configuration.maximum_value,
                                        size=self.configuration.genome_size)


class GrammarRepresentation(DirectRepresentation):

    def __init__(self, alphabet: type(Alphabet)):
        self.alphabet: Alphabet = alphabet
        super().__init__(Alphabet)

    def _initialize(self):
        self.genome = random.choices(self.alphabet.list(), k=self.configuration.genome_size)

    @abstractmethod
    def compatibility(self, other):
        pass
