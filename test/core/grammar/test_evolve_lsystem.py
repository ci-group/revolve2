import copy
import unittest

import numpy as np

from nca.core.evolution.conditions.initialization import Initialization
from nca.core.genome.grammar.lsystem_representation import LSystemRepresentation
from nca.core.genome.initialization import UniformInitialization
from nca.core.genome.operators.mutation_operator import SwapMutation, InversionMutation, InsertMutation, ReplaceMutation
from nca.core.genome.operators.recombination_operator import OnePointCrossover
from test.core.grammar.test_alphabet import TestColorAlphabet


TestRules = {TestColorAlphabet.GREEN: [TestColorAlphabet.RED, TestColorAlphabet.RED],
             TestColorAlphabet.BLUE: [TestColorAlphabet.RED, TestColorAlphabet.RED]}


class LSystemRepresentationTest(unittest.TestCase):

    def test_swap_mutation(self):
        mutation = SwapMutation()

        representation = LSystemRepresentation(TestColorAlphabet, TestRules)
        representation.genome =[TestColorAlphabet.RED, TestColorAlphabet.GREEN, TestColorAlphabet.BLUE]

        new_representation = copy.deepcopy(representation)
        mutation._execute(new_representation, Initialization())

        self.assertTrue(len(representation.genome) > 0)
        self.assertNotEqual(representation.genome, new_representation.genome)

    def test_inversion_mutation(self):
        mutation = InversionMutation()

        representation = LSystemRepresentation(TestColorAlphabet, TestRules)
        representation.genome = [TestColorAlphabet.RED, TestColorAlphabet.GREEN, TestColorAlphabet.BLUE]

        new_representation = copy.deepcopy(representation)
        mutation._execute(new_representation, Initialization())

        self.assertTrue(len(representation.genome) > 0)
        self.assertNotEqual(representation.genome, new_representation.genome)

    def test_insert_mutation(self):
        mutation = InsertMutation()

        representation = LSystemRepresentation(TestColorAlphabet, TestRules)

        new_representation = copy.deepcopy(representation)
        mutation._execute(new_representation, UniformInitialization())

        self.assertTrue(len(representation.genome) > 0)
        self.assertTrue(len(new_representation.genome) > len(representation.genome))

    def test_replace_mutation(self):
        mutation = ReplaceMutation()

        representation = LSystemRepresentation(TestColorAlphabet, TestRules)

        new_representation = copy.deepcopy(representation)
        mutation._execute(new_representation, UniformInitialization())

        self.assertTrue(len(representation.genome) > 0)
        self.assertTrue(len(new_representation.genome) == len(representation.genome))
        self.assertNotEqual(representation.genome, new_representation.genome)

    def test_crossover_recombination(self):
        recombination = OnePointCrossover()

        representation_1 = LSystemRepresentation(TestColorAlphabet, TestRules)
        representation_1.genome = [TestColorAlphabet.RED, TestColorAlphabet.GREEN, TestColorAlphabet.BLUE]
        representation_2 = LSystemRepresentation(TestColorAlphabet, TestRules)
        representation_2.genome = [TestColorAlphabet.BLUE, TestColorAlphabet.GREEN, TestColorAlphabet.RED]

        new_representation = recombination._execute([representation_1, representation_2])

        self.assertNotEqual(representation_1.genome, new_representation.genome)
        self.assertNotEqual(representation_2.genome, new_representation.genome)
