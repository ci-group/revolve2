import unittest

from nca.core.genome.representations.chromosomal_representation import ChromosomalRepresentation
from nca.core.genome.initialization import UniformInitialization, BinaryInitialization, IntegerInitialization, \
    GaussianInitialization


class ChromosomalRepresentationTest(unittest.TestCase):

    def test_binary(self):
        initialization = BinaryInitialization()
        representation_1 = ChromosomalRepresentation(initialization)
        representation_2 = ChromosomalRepresentation(initialization)

        self.assertNotEqual(representation_1, representation_2)

        compatibility = representation_1.compatibility(representation_2)
        self.assertNotEqual(compatibility, 0.0)

    def test_integer(self):
        initialization = IntegerInitialization()
        representation_1 = ChromosomalRepresentation(initialization)
        representation_2 = ChromosomalRepresentation(initialization)

        self.assertNotEqual(representation_1, representation_2)

        compatibility = representation_1.compatibility(representation_2)
        self.assertNotEqual(compatibility, 0.0)

    def test_uniform(self):
        initialization = UniformInitialization()
        representation_1 = ChromosomalRepresentation(initialization)
        representation_2 = ChromosomalRepresentation(initialization)

        self.assertNotEqual(representation_1, representation_2)

        compatibility = representation_1.compatibility(representation_2)
        self.assertNotEqual(compatibility, 0.0)

    def test_gaussian(self):
        initialization = GaussianInitialization()
        representation_1 = ChromosomalRepresentation(initialization)
        representation_2 = ChromosomalRepresentation(initialization)

        self.assertNotEqual(representation_1, representation_2)

        compatibility = representation_1.compatibility(representation_2)
        self.assertNotEqual(compatibility, 0.0)
