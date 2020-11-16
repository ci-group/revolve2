import copy
import unittest

from nca.core.abstract.structural.tree.tree_helper import Orientation
from nca.core.genome.operators.mutation_operator import SwapMutation, ReplaceMutation, InversionMutation, InsertMutation
from revolve.robot.robogen.robogen_grammar import RobogenSymbol
from revolve.robot.robogen.robogen_genotype import IndirectRobogenGenotype


class TestRobogenManipulation(unittest.TestCase):

    def test_swap_mutation(self):
        mutation = SwapMutation()

        genotype = IndirectRobogenGenotype()

        new_genotype = copy.deepcopy(genotype)
        mutation._mutate(new_genotype.get_random_representation())

        self.assertTrue(len(new_genotype) == len(genotype))
        #self.assertNotEqual(genotype, new_genotype) # swaps can be swapping to of the same symbols

    def test_inversion_mutation(self):
        mutation = InversionMutation()

        genotype = IndirectRobogenGenotype()

        new_genotype = copy.deepcopy(genotype)
        mutation._mutate(new_genotype.get_random_representation())

        self.assertTrue(len(new_genotype) > 0)
        self.assertNotEqual(genotype, new_genotype)

    def test_insert_mutation(self):
        mutation = InsertMutation()

        genotype = IndirectRobogenGenotype()

        new_genotype = copy.deepcopy(genotype)
        mutation._mutate(new_genotype.get_random_representation())

        self.assertTrue(len(new_genotype) > 0)
        self.assertNotEqual(genotype, new_genotype)

    def test_replace_mutation(self):
        mutation = ReplaceMutation()

        genotype = IndirectRobogenGenotype()

        new_genotype = copy.deepcopy(genotype)
        mutation._mutate(new_genotype.get_random_representation())

        self.assertTrue(len(new_genotype) > 0)
        self.assertNotEqual(genotype, new_genotype)
