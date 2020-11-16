import copy
import unittest

from nca.core.genome.genotype import Genotype
from nca.core.genome.operators.initialization import UniformInitialization
from nca.core.genome.operators.mutation_operator import SwapMutation, InversionMutation, InsertMutation, \
    ReplaceMutation, DeleteMutation
from nca.core.genome.representations.valued_representation import ValuedRepresentation


class TestMutationOperators(unittest.TestCase):

    def test_mutation_operator(self):
        mutation = SwapMutation()

        genotype = Genotype(ValuedRepresentation(UniformInitialization()))
        new_genotype = mutation(copy.deepcopy(genotype), debug=True)

        self.assertNotEqual(genotype, new_genotype)

    def test_swap_mutation(self):
        mutation = SwapMutation()

        representation = ValuedRepresentation(UniformInitialization())
        self.assertTrue(len(representation) > 0)

        new_representation = copy.deepcopy(representation)
        mutation._mutate(new_representation)

        self.assertTrue(len(representation) == len(new_representation))
        self.assertNotEqual(representation, new_representation)

    def test_delete_mutation(self):
        mutation = DeleteMutation()

        representation = ValuedRepresentation(UniformInitialization())

        new_representation = copy.deepcopy(representation)
        mutation._mutate(new_representation)

        self.assertTrue(len(representation) == len(new_representation) + 1)
        self.assertNotEqual(representation, new_representation)

    def test_inversion_mutation(self):
        mutation = InversionMutation()

        representation = ValuedRepresentation(UniformInitialization())

        new_representation = copy.deepcopy(representation)
        mutation._mutate(new_representation)

        self.assertTrue(len(representation) > 0)
        self.assertNotEqual(representation, new_representation)

    def test_insert_mutation(self):
        mutation = InsertMutation()

        representation = ValuedRepresentation(UniformInitialization())

        new_representation = copy.deepcopy(representation)
        mutation._mutate(new_representation)

        self.assertTrue(len(representation) > 0)
        self.assertTrue(len(new_representation) > len(representation))

    def test_replace_mutation(self):
        mutation = ReplaceMutation()

        representation = ValuedRepresentation(UniformInitialization())

        new_representation = copy.deepcopy(representation)
        mutation._mutate(new_representation)

        self.assertTrue(len(representation) > 0)
        self.assertTrue(len(new_representation) == len(representation))
        self.assertNotEqual(representation, new_representation)
