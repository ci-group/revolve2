import unittest

from nca.core.genome.operators.recombination_operator import OnePointCrossover
from nca.core.genome.representations.direct_representation import RealValuedRepresentation


class TestRecombinationOperators(unittest.TestCase):

    def test_crossover_recombination(self):
        recombination = OnePointCrossover()

        representation_1 = RealValuedRepresentation()
        representation_2 = RealValuedRepresentation()

        new_representation = recombination._execute([representation_1, representation_2])
        self.assertNotEqual(representation_1.genome, new_representation.genome)
        self.assertNotEqual(representation_2.genome, new_representation.genome)
