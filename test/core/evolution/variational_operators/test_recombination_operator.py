import copy
import unittest

import numpy as np

from nca.core.genome.operators.recombination_operator import OnePointCrossover
from nca.core.genome.representations.direct_representation import RealValuedRepresentation


class TestRecombinationOperators(unittest.TestCase):

    def test_crossover_recombination(self):
        recombination = OnePointCrossover()

        representation_1 = RealValuedRepresentation()
        representation_2 = RealValuedRepresentation()

        new_representation_1 = copy.deepcopy(representation_1)
        new_representation_2 = copy.deepcopy(representation_2)
        recombination._execute([new_representation_1, new_representation_2])

        self.assertFalse(np.allclose(new_representation_1.genome, representation_2.genome))
        self.assertFalse(np.allclose(new_representation_1.genome, representation_2.genome))