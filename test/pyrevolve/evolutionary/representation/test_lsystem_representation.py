import unittest

from pyrevolve.evolutionary.algorithm.genome.operators.mutation_operator import MutationOperator
from pyrevolve.evolutionary.algorithm.genome.operators.recombination_operator import RecombinationOperator
from pyrevolve.evolutionary.algorithm.genome.representations.l_system.alphabet import TestColorAlphabet
from pyrevolve.evolutionary.algorithm.genome.representations.l_system.lsystem_representation import \
    LSystemRepresentation
from pyrevolve.shared.configurations import RepresentationConfiguration


class LSystemRepresentationTest(unittest.TestCase):

    def test_same(self):
        alphabet = TestColorAlphabet

        rules = {alphabet.GREEN: alphabet.RED, alphabet.BLUE: alphabet.RED}

        representation = LSystemRepresentation(alphabet, rules)
        representation.apply_rules()

        outcome = [alphabet.RED for _ in range(RepresentationConfiguration().genome_size)]

        self.assertEqual(representation.genome, outcome)

