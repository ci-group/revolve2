import copy
import unittest

import numpy as np

from pyrevolve.evolutionary.algorithm.genome.operators.mutation_operator import SwapMutation, InversionMutation
from pyrevolve.evolutionary.algorithm.genome.operators.recombination_operator import RecombinationOperator, \
    OnePointCrossover
from pyrevolve.evolutionary.algorithm.genome.representations.direct_representation import RealValuedRepresentation


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