import copy
import unittest

from revolve2.nca.core.genome.operators.mutation_operator import SwapMutation, DeleteMutation, InversionMutation, InsertMutation, \
    ReplaceMutation
from revolve2.revolve.robot.body.robogen.robogen_genotype import IndirectRobogenGenotype


class TestMutationOperators(unittest.TestCase):

    def test_swap_mutation(self):
        mutation = SwapMutation()
        self.check_mutation(mutation)

    def test_delete_mutation(self):
        mutation = DeleteMutation()
        self.check_mutation(mutation)

    def test_inversion_mutation(self):
        mutation = InversionMutation()
        self.check_mutation(mutation)

    def test_insert_mutation(self):
        mutation = InsertMutation()
        self.check_mutation(mutation)

    def test_replace_mutation(self):
        mutation = ReplaceMutation()
        self.check_mutation(mutation)

    def check_mutation(self, mutation):
        genotype = IndirectRobogenGenotype()
        representation = genotype.get_random_representation()
        self.assertTrue(len(genotype) > 0)

        new_genotype = copy.deepcopy(genotype)
        self.assertEqual(genotype, new_genotype)

        mutation._mutate(representation)
        self.assertTrue(len(genotype) == len(new_genotype))
        self.assertNotEqual(genotype, new_genotype)
