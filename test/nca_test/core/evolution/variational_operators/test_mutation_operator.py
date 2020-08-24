import copy
import unittest

from nca.core.evolution.conditions.initialization import Initialization
from nca.core.genome.initialization import UniformInitialization
from nca.core.genome.operators.mutation_operator import SwapMutation, InversionMutation, InsertMutation, \
    ReplaceMutation, DeleteMutation
from nca.core.genome.representations.valued_representation import ValuedRepresentation


class TestMutationOperators(unittest.TestCase):

    def test_swap_mutation(self):
        mutation = SwapMutation()

        representation = ValuedRepresentation(UniformInitialization())

        new_representation = copy.deepcopy(representation)
        mutation._mutate(new_representation)

        self.assertTrue(len(representation.genome) > 0)
        self.assertNotEqual(representation.genome, new_representation.genome)

    def test_delete_mutation(self):
        mutation = DeleteMutation()

        representation = ValuedRepresentation(UniformInitialization())

        new_representation = copy.deepcopy(representation)
        mutation._mutate(new_representation)

        self.assertTrue(len(representation.genome) == len(new_representation.genome) + 1)
        self.assertNotEqual(representation.genome, new_representation.genome)

    def test_inversion_mutation(self):
        mutation = InversionMutation()

        representation = ValuedRepresentation(UniformInitialization())

        new_representation = copy.deepcopy(representation)
        mutation._mutate(new_representation)

        self.assertTrue(len(representation.genome) > 0)
        self.assertNotEqual(representation.genome, new_representation.genome)

    def test_insert_mutation(self):
        mutation = InsertMutation()

        representation = ValuedRepresentation(UniformInitialization())

        new_representation = copy.deepcopy(representation)
        mutation._mutate(new_representation)

        self.assertTrue(len(representation.genome) > 0)
        self.assertTrue(len(new_representation.genome) > len(representation.genome))

    def test_replace_mutation(self):
        mutation = ReplaceMutation()

        representation = ValuedRepresentation(UniformInitialization())

        new_representation = copy.deepcopy(representation)
        mutation._mutate(new_representation)

        self.assertTrue(len(representation.genome) > 0)
        self.assertTrue(len(new_representation.genome) == len(representation.genome))
        self.assertNotEqual(representation.genome, new_representation.genome)
