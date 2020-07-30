import random
from abc import ABC, abstractmethod

import numpy as np

from pyrevolve.evolutionary.algorithm.genome.representation import Representation
from pyrevolve.evolutionary.algorithm.genome.representation_visitor import RepresentationVisitor
from pyrevolve.evolutionary.algorithm.genome.representations.l_system.alphabet import Alphabet


class DirectRepresentation(Representation, ABC):

    def __init__(self):
        super().__init__()

    @abstractmethod
    def compatibility(self, other):
        pass

    @abstractmethod
    def visit(self, visitor: RepresentationVisitor):
        pass


class ValuedRepresentation(Representation, ABC):

    def __init__(self, data_type: type):
        super().__init__()
        self.data_type = data_type
        self.init(None)

    def compatibility(self, other):
        return np.linalg.norm(self.genome - other.genome)

    def visit(self, visitor: RepresentationVisitor):
        visitor.visit_valued_representation(self)


class BinaryRepresentation(ValuedRepresentation):

    def __init__(self):
        super().__init__(bool)

    def _initialize(self):
        self.genome = np.random.randint(2, size=self.configuration.genome_size)


class IntegerRepresentation(ValuedRepresentation):

    def __init__(self):
        super().__init__(int)

    def _initialize(self):
        self.genome = np.random.randint(self.configuration.minimum_value, self.configuration.maximum_value,
                                        size=self.configuration.genome_size)


class RealValuedRepresentation(ValuedRepresentation):

    def __init__(self):
        super().__init__(float)

    def _initialize(self):
        self.genome = np.random.uniform(self.configuration.minimum_value, self.configuration.maximum_value,
                                        size=self.configuration.genome_size)


class GrammarRepresentation(Representation):

    def __init__(self, alphabet: type(Alphabet)):
        super().__init__()
        self.alphabet: Alphabet = alphabet
        self.init(None)

    def _initialize(self):
        self.genome = random.choices(self.alphabet.list(), k=self.configuration.genome_size)

    def compatibility(self, other):
        differences = 0

        for index, element in enumerate(self.genome):
            if self.genome[index] != other.genome[index]:
                differences += 1

        return differences

    def visit(self, visitor: RepresentationVisitor):
        visitor.visit_grammar_representation(self)
