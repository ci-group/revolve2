import unittest

from nca.experiment.configurations import RepresentationConfiguration
from nca.core.genome.representations.l_system.alphabet import TestColorAlphabet
from nca.core.genome.representations.l_system.lsystem_representation import \
    LSystemRepresentation


class LSystemRepresentationTest(unittest.TestCase):

    def test_same(self):
        alphabet = TestColorAlphabet

        rules = {alphabet.GREEN: alphabet.RED, alphabet.BLUE: alphabet.RED}

        representation = LSystemRepresentation(alphabet, rules)
        representation.apply_rules()

        outcome = [alphabet.RED for _ in range(RepresentationConfiguration().genome_size)]

        self.assertEqual(representation.genome, outcome)

