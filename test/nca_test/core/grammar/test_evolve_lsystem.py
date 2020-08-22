import copy
import unittest

from nca.core.evolution.conditions.initialization import Initialization
from nca.core.genome.grammar.grammar import ReplacementRules, Grammar
from nca.core.genome.initialization import UniformInitialization
from nca.core.genome.operators.mutation_operator import SwapMutation, InversionMutation, InsertMutation, \
    ReplaceMutation, DeleteMutation
from nca.core.genome.operators.recombination_operator import OnePointCrossover
from nca.core.genome.representations.symbolic_representation import LSystemRepresentation
from nca_test.core.grammar.test_alphabet import TestColorSymbol


TestRules: ReplacementRules = {TestColorSymbol.GREEN: [[TestColorSymbol.RED, TestColorSymbol.RED]],
                               TestColorSymbol.BLUE: [[TestColorSymbol.RED, TestColorSymbol.RED]]}


class LSystemRepresentationTest(unittest.TestCase):

    def test_swap_mutation(self):
        mutation = SwapMutation()

        representation = LSystemRepresentation(Grammar(TestColorSymbol, TestRules))
        representation.genome = [TestColorSymbol.RED, TestColorSymbol.GREEN, TestColorSymbol.BLUE]

        new_representation = copy.deepcopy(representation)
        mutation._mutate(new_representation)

        self.assertTrue(len(representation.genome) > 0)
        self.assertNotEqual(representation.genome, new_representation.genome)

    def test_inversion_mutation(self):
        mutation = InversionMutation()

        representation = LSystemRepresentation(Grammar(TestColorSymbol, TestRules))
        representation.genome = [TestColorSymbol.RED, TestColorSymbol.GREEN, TestColorSymbol.BLUE]

        new_representation = copy.deepcopy(representation)
        mutation._mutate(new_representation)

        self.assertTrue(len(representation.genome) > 0)
        self.assertNotEqual(representation.genome, new_representation.genome)

    def test_insert_mutation(self):
        mutation = InsertMutation()

        representation = LSystemRepresentation(Grammar(TestColorSymbol, TestRules))

        new_representation = copy.deepcopy(representation)
        mutation._mutate(new_representation)

        self.assertTrue(len(representation.genome) > 0)
        self.assertTrue(len(new_representation.genome) > len(representation.genome))

    def test_replace_mutation(self):
        mutation = ReplaceMutation()

        representation = LSystemRepresentation(Grammar(TestColorSymbol, TestRules))

        new_representation = copy.deepcopy(representation)
        mutation._mutate(new_representation)

        self.assertTrue(len(representation.genome) > 0)
        self.assertTrue(len(new_representation.genome) == len(representation.genome))
        #self.assertNotEqual(representation.genome, new_representation.genome) # cannot be guaranteed due to replacing with the same symbol.

    def test_delete_mutation(self):
        mutation = DeleteMutation()

        representation = LSystemRepresentation(Grammar(TestColorSymbol, TestRules))

        new_representation = copy.deepcopy(representation)
        mutation._mutate(new_representation)

        self.assertTrue(len(representation.genome) > 0)
        self.assertTrue(len(new_representation.genome) < len(representation.genome))
        self.assertNotEqual(representation.genome, new_representation.genome)

    def test_crossover_recombination(self):
        recombination = OnePointCrossover()

        representation_1 = LSystemRepresentation(Grammar(TestColorSymbol, TestRules))
        representation_1.genome = [TestColorSymbol.RED, TestColorSymbol.GREEN, TestColorSymbol.BLUE]
        representation_2 = LSystemRepresentation(Grammar(TestColorSymbol, TestRules))
        representation_2.genome = [TestColorSymbol.BLUE, TestColorSymbol.GREEN, TestColorSymbol.RED]

        new_representation = recombination._recombine([representation_1, representation_2])

        self.assertNotEqual(representation_1.genome, new_representation.genome)
        self.assertNotEqual(representation_2.genome, new_representation.genome)
