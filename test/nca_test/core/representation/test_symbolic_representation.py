import unittest

from nca.core.genome.grammar.grammar import Grammar, ReplacementRules
from nca.core.genome.representations.symbolic_representation import LSystemRepresentation
from nca_test.core.grammar.test_alphabet import TestColorSymbol


class DirectRepresentationTest(unittest.TestCase):

    def test_lsystem(self):
        representation_1 = LSystemRepresentation(Grammar(TestColorSymbol, ReplacementRules))
        representation_2 = LSystemRepresentation(Grammar(TestColorSymbol, ReplacementRules))

        self.assertNotEqual(representation_1, representation_2)

        compatibility = representation_1.compatibility(representation_2)
        self.assertNotEqual(compatibility, 0.0)
