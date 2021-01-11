import copy
import unittest

from nca.core.genome.operators.initialization import UniformInitialization
from nca.core.genome.operators.mutation_operator import SwapMutation, InversionMutation, InsertMutation, \
    ReplaceMutation, DeleteMutation
from nca.core.genome.representations.valued_representation import ValuedRepresentation

"""
class TestMutationOperators(unittest.TestCase):

    def test_swap_mutation(self):
        mutation = SwapMutation()

        representation = ChromosomalRepresentation(UniformInitialization())

        new_representation = copy.deepcopy(representation)
        mutation._mutate(new_representation)

        self.assertTrue(len(representation) > 0)
        self.assertNotEqual(representation, new_representation)

    def test_inversion_mutation(self):
        mutation = InversionMutation()

        representation = ChromosomalRepresentation(UniformInitialization())

        new_representation = copy.deepcopy(representation)
        mutation._mutate(new_representation)

        self.assertTrue(len(representation) > 0)
        self.assertNotEqual(representation, new_representation)

    def test_insert_mutation(self):
        mutation = InsertMutation()

        representation = ChromosomalRepresentation(UniformInitialization())

        new_representation = copy.deepcopy(representation)
        mutation._mutate(new_representation)

        self.assertTrue(len(representation) > 0)
        self.assertTrue(len(new_representation) > len(representation))

    def test_replace_mutation(self):
        mutation = ReplaceMutation()

        representation = ChromosomalRepresentation(UniformInitialization())

        new_representation = copy.deepcopy(representation)
        mutation._mutate(new_representation)

        self.assertTrue(len(representation) > 0)
        self.assertTrue(len(new_representation) == len(representation))
        self.assertNotEqual(representation, new_representation)
"""