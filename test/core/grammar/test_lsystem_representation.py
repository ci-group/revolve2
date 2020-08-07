import unittest

from nca.core.abstract.configurations import RepresentationConfiguration
from nca.core.genome.grammar.lsystem_representation import LSystemRepresentation
from test.core.grammar.test_alphabet import TestColorAlphabet


class LSystemRepresentationTest(unittest.TestCase):

    def test_same(self):
        alphabet = TestColorAlphabet

        rules = {alphabet.GREEN: [alphabet.RED], alphabet.BLUE: [alphabet.RED]}

        representation = LSystemRepresentation(alphabet, rules)
        representation.apply_rules()

        outcome = [alphabet.RED for _ in range(RepresentationConfiguration().genome_size)]
        self.assertEqual(representation.genome, outcome)

    def test_array(self):
        alphabet = TestColorAlphabet

        rules = {alphabet.GREEN: [alphabet.RED, alphabet.RED], alphabet.BLUE: [alphabet.RED, alphabet.RED]}

        representation = LSystemRepresentation(alphabet, rules)
        representation.apply_rules()

        outcome = [alphabet.RED for _ in range(len(representation.genome))]
        self.assertEqual(representation.genome, outcome)
