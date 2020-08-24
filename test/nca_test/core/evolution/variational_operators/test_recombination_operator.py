import unittest

from nca.core.genome.initialization import UniformInitialization
from nca.core.genome.operators.recombination_operator import OnePointCrossover
from nca.core.genome.representations.valued_representation import ValuedRepresentation


class TestRecombinationOperators(unittest.TestCase):

    def test_crossover_recombination(self):
        recombination = OnePointCrossover()
        initialization = UniformInitialization()

        representation_1 = ValuedRepresentation(initialization)
        representation_2 = ValuedRepresentation(initialization)

        new_representation = recombination._recombine([representation_1, representation_2])
        self.assertNotEqual(representation_1.genome, new_representation.genome)
        self.assertNotEqual(representation_2.genome, new_representation.genome)
