import copy
import unittest

import numpy as np

from pyrevolve.evolutionary.algorithm.conditions.initialization import Initialization
from pyrevolve.evolutionary.algorithm.genome.initialization import UniformInitialization
from pyrevolve.evolutionary.algorithm.genome.operators.mutation_operator import SwapMutation, InversionMutation, \
    InsertMutation, ReplaceMutation
from pyrevolve.evolutionary.algorithm.genome.operators.recombination_operator import RecombinationOperator
from pyrevolve.evolutionary.algorithm.genome.representations.direct_representation import RealValuedRepresentation


class TestMutationOperators(unittest.TestCase):

    def test_swap_mutation(self):
        mutation = SwapMutation()

        representation = RealValuedRepresentation()

        new_representation = copy.deepcopy(representation)
        mutation._execute(new_representation, Initialization())

        self.assertTrue(len(representation.genome) > 0)
        self.assertFalse(np.allclose(new_representation.genome, representation.genome))

    def test_inversion_mutation(self):
        mutation = InversionMutation()

        representation = RealValuedRepresentation()

        new_representation = copy.deepcopy(representation)
        mutation._execute(new_representation, Initialization())

        self.assertTrue(len(representation.genome) > 0)
        self.assertFalse(np.allclose(new_representation.genome, representation.genome))

    def test_insert_mutation(self):
        mutation = InsertMutation()

        representation = RealValuedRepresentation()

        new_representation = copy.deepcopy(representation)
        mutation._execute(new_representation, UniformInitialization())

        self.assertTrue(len(representation.genome) > 0)
        self.assertTrue(len(new_representation.genome) > len(representation.genome))

    def test_replace_mutation(self):
        mutation = ReplaceMutation()

        representation = RealValuedRepresentation()

        new_representation = copy.deepcopy(representation)
        mutation._execute(new_representation, UniformInitialization())

        self.assertTrue(len(representation.genome) > 0)
        self.assertTrue(len(new_representation.genome) == len(representation.genome))
        self.assertFalse(np.allclose(new_representation.genome, representation.genome))
