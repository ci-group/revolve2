import unittest

from nca.core.genome.operators.initialization import UniformInitialization, BinaryInitialization, IntegerInitialization, \
    GaussianInitialization
from nca.core.genome.representations.valued_representation import ValuedRepresentation


class DirectRepresentationTest(unittest.TestCase):

    def test_binary(self):
        initialization = BinaryInitialization()
        representation_1 = ValuedRepresentation(initialization)
        representation_2 = ValuedRepresentation(initialization)

        self.assertNotEqual(representation_1, representation_2)

        compatibility = representation_1.compatibility(representation_2)
        self.assertNotEqual(compatibility, 0.0)

    def test_integer(self):
        initialization = IntegerInitialization()
        representation_1 = ValuedRepresentation(initialization)
        representation_2 = ValuedRepresentation(initialization)

        self.assertNotEqual(representation_1, representation_2)

        compatibility = representation_1.compatibility(representation_2)
        self.assertNotEqual(compatibility, 0.0)

    def test_uniform(self):
        initialization = UniformInitialization()
        representation_1 = ValuedRepresentation(initialization)
        representation_2 = ValuedRepresentation(initialization)

        self.assertNotEqual(representation_1, representation_2)

        compatibility = representation_1.compatibility(representation_2)
        self.assertNotEqual(compatibility, 0.0)

    def test_gaussian(self):
        initialization = GaussianInitialization()
        representation_1 = ValuedRepresentation(initialization)
        representation_2 = ValuedRepresentation(initialization)

        self.assertNotEqual(representation_1, representation_2)

        compatibility = representation_1.compatibility(representation_2)
        self.assertNotEqual(compatibility, 0.0)
