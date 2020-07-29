import copy
import unittest

import numpy as np

from pyrevolve.evolutionary.algorithm.genome.operators.mutation_operator import SwapMutation, InversionMutation
from pyrevolve.evolutionary.algorithm.genome.operators.recombination_operator import RecombinationOperator
from pyrevolve.evolutionary.algorithm.genome.representations.direct_representation import RealValuedRepresentation


class TestMutationOperators(unittest.TestCase):

    def test_swap_mutation(self):
        mutation = SwapMutation()

        representation = RealValuedRepresentation()

        new_representation = copy.deepcopy(representation)
        mutation._execute(new_representation)

        self.assertFalse(np.allclose(new_representation.genome, representation.genome))

    def test_inversion_mutation(self):
        mutation = InversionMutation()

        representation = RealValuedRepresentation()

        new_representation = copy.deepcopy(representation)
        mutation._execute(new_representation)

        self.assertFalse(np.allclose(new_representation.genome, representation.genome))
