import unittest

import numpy as np

from pyrevolve.evolutionary.algorithm.genome.operators.mutation_operator import MutationOperator
from pyrevolve.evolutionary.algorithm.genome.operators.recombination_operator import RecombinationOperator
from pyrevolve.evolutionary.algorithm.genome.representation_builder import RepresentationBuilder
from pyrevolve.evolutionary.algorithm.genome.representations.direct_representation import BinaryRepresentation, \
    RealValuedRepresentation, IntegerRepresentation


class RepresentationBuilderTest(unittest.TestCase):

    def test_same(self):
        representation_builder = RepresentationBuilder(BinaryRepresentation, MutationOperator(), RecombinationOperator())

        representation_1 = representation_builder.generate()
        representation_2 = representation_builder.generate()

        representation_2.init(representation_1.genome)

        self.assertTrue(np.allclose(representation_1.genome, representation_2.genome))

    def test_binary(self):
        representation_builder = RepresentationBuilder(BinaryRepresentation, MutationOperator(),
                                                       RecombinationOperator())

        representation_1 = representation_builder.generate()
        representation_2 = representation_builder.generate()

        self.assertFalse(np.allclose(representation_1.genome, representation_2.genome))

    def test_integer(self):
        representation_builder = RepresentationBuilder(IntegerRepresentation, MutationOperator(),
                                                       RecombinationOperator())

        representation_1 = representation_builder.generate()
        representation_2 = representation_builder.generate()

        self.assertFalse(np.allclose(representation_1.genome, representation_2.genome))

    def test_real_valued(self):
        representation_builder = RepresentationBuilder(RealValuedRepresentation, MutationOperator(),
                                                       RecombinationOperator())

        representation_1 = representation_builder.generate()
        representation_2 = representation_builder.generate()

        self.assertFalse(np.allclose(representation_1.genome, representation_2.genome))
