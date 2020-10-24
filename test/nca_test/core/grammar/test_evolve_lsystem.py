import copy
import unittest

from nca.core.actor.individual import Individual
from nca.core.evolution.conditions.initialization import Initialization
from nca.core.genome.grammar.grammar import ReplacementRules, RewritingGrammar
from nca.core.genome.grammar.grammar_initialization import GrammarInitialization
from nca.core.genome.initialization import UniformInitialization
from nca.core.genome.operators.mutation_operator import SwapMutation, InversionMutation, InsertMutation, \
    ReplaceMutation, DeleteMutation
from nca.core.genome.operators.recombination_operator import OnePointCrossover
from nca.core.genome.representations.symbolic_representation import LSystemAlgorithm, SymbolicRepresentation
from nca_test.core.grammar.test_alphabet import TestColorSymbol


TestRules: ReplacementRules = {TestColorSymbol.GREEN: [[TestColorSymbol.RED, TestColorSymbol.RED]],
                               TestColorSymbol.BLUE: [[TestColorSymbol.RED, TestColorSymbol.RED]]}


class LSystemRepresentationTest(unittest.TestCase):

    def test_swap_mutation(self):
        mutation = SwapMutation()

        algorithm = LSystemAlgorithm(RewritingGrammar(TestRules))
        algorithm.representation.extend([TestColorSymbol.RED, TestColorSymbol.GREEN, TestColorSymbol.BLUE])
        self.assertTrue(len(algorithm.representation) > 0)

        new_representation = copy.deepcopy(algorithm.representation)
        mutation._mutate(new_representation)

        self.assertTrue(len(algorithm.representation) == len(new_representation))
        #self.assertNotEqual(representation, new_representation) # can be the same

    def test_inversion_mutation(self):
        mutation = InversionMutation()

        algorithm = LSystemAlgorithm(RewritingGrammar(TestRules), None)
        algorithm.representation.extend([TestColorSymbol.RED, TestColorSymbol.GREEN, TestColorSymbol.BLUE])
        self.assertTrue(len(algorithm.representation) > 0)

        new_representation = copy.deepcopy(algorithm.representation)
        mutation._mutate(new_representation)

        self.assertTrue(len(algorithm.representation) == len(new_representation))
        #self.assertNotEqual(representation, new_representation)

    def test_insert_mutation(self):
        mutation = InsertMutation()

        algorithm = LSystemAlgorithm(RewritingGrammar(TestRules), GrammarInitialization(TestColorSymbol))

        new_representation = copy.deepcopy(algorithm.representation)
        mutation._mutate(new_representation)

        self.assertTrue(len(algorithm.representation) > 0)
        self.assertTrue(len(new_representation) > len(algorithm.representation))
        self.assertNotEqual(algorithm.representation, new_representation)

    def test_replace_mutation(self):
        mutation = ReplaceMutation()

        algorithm = LSystemAlgorithm(RewritingGrammar(TestRules), GrammarInitialization(TestColorSymbol))

        new_representation = copy.deepcopy(algorithm.representation)
        mutation._mutate(new_representation)

        self.assertTrue(len(algorithm.representation) > 0)
        self.assertTrue(len(new_representation) == len(algorithm.representation))
        #self.assertNotEqual(representation.genome, new_representation.genome) # cannot be guaranteed due to replacing with the same symbol.

    def test_delete_mutation(self):
        mutation = DeleteMutation()

        algorithm = LSystemAlgorithm(RewritingGrammar(TestRules), GrammarInitialization(TestColorSymbol))

        new_representation = copy.deepcopy(algorithm.representation)
        mutation._mutate(new_representation)

        self.assertTrue(len(algorithm.representation) > 0)
        self.assertTrue(len(new_representation) < len(algorithm.representation))
        self.assertNotEqual(algorithm.representation, new_representation)

    def test_crossover_recombination(self):
        recombination = OnePointCrossover()

        representation_1 = SymbolicRepresentation()
        representation_1[:] = [TestColorSymbol.RED, TestColorSymbol.GREEN, TestColorSymbol.BLUE]
        representation_2 = SymbolicRepresentation()
        representation_2[:] = [TestColorSymbol.BLUE, TestColorSymbol.GREEN, TestColorSymbol.RED]

        copy_representation_1 = copy.deepcopy(representation_1)
        copy_representation_2 = copy.deepcopy(representation_2)
        new_representation = recombination._recombine([copy_representation_1, copy_representation_2])

        self.assertNotEqual(representation_1, new_representation)
        self.assertNotEqual(representation_2, new_representation)
