import copy
import unittest

from nca.core.evolution.conditions.initialization import Initialization
from nca.core.genome.initialization import UniformInitialization
from nca.core.genome.operators.mutation_operator import SwapMutation, InversionMutation, InsertMutation, ReplaceMutation
from nca.core.genome.representations.direct_representation import RealValuedRepresentation


class TestMutationOperators(unittest.TestCase):

    def test_swap_mutation(self):
        mutation = SwapMutation()

        representation = RealValuedRepresentation()

        new_representation = copy.deepcopy(representation)
        mutation._execute(new_representation, Initialization())

        self.assertTrue(len(representation.genome) > 0)
        self.assertNotEqual(representation.genome, new_representation.genome)

    def test_inversion_mutation(self):
        mutation = InversionMutation()

        representation = RealValuedRepresentation()

        new_representation = copy.deepcopy(representation)
        mutation._execute(new_representation, Initialization())

        self.assertTrue(len(representation.genome) > 0)
        self.assertNotEqual(representation.genome, new_representation.genome)

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
        self.assertNotEqual(representation.genome, new_representation.genome)
