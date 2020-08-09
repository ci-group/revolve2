import random
from abc import ABC, abstractmethod

import numpy as np

from nca.core.genome.representation import Representation


class DirectRepresentation(Representation, ABC):

    def __init__(self):
        super().__init__()

    @abstractmethod
    def compatibility(self, other) -> float:
        pass

    @abstractmethod
    def visit(self, representation_visitor):
        pass


class ValuedRepresentation(DirectRepresentation, ABC):

    def __init__(self, data_type: type):
        super().__init__()
        self.data_type = data_type
        self.init()

    def compatibility(self, other) -> float:
        return np.linalg.norm(np.array(self.genome) - np.array(other.genome))

    def visit(self, representation_visitor):
        representation_visitor.visit_valued_representation(self)


class BinaryRepresentation(ValuedRepresentation):

    def __init__(self):
        super().__init__(bool)

    def _initialize(self):
        self.genome = np.random.randint(2, size=self.configuration.genome_size).tolist()


class IntegerRepresentation(ValuedRepresentation):

    def __init__(self):
        super().__init__(int)

    def _initialize(self):
        self.genome = np.random.randint(self.configuration.minimum_value, self.configuration.maximum_value,
                                        size=self.configuration.genome_size).tolist()


class RealValuedRepresentation(ValuedRepresentation):

    def __init__(self):
        super().__init__(float)

    def _initialize(self):
        self.genome = np.random.uniform(self.configuration.minimum_value, self.configuration.maximum_value,
                                        size=self.configuration.genome_size).tolist()
