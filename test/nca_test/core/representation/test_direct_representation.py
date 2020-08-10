import unittest

from nca.core.genome.representations.direct_representation import BinaryRepresentation, IntegerRepresentation, \
    RealValuedRepresentation


class DirectRepresentationTest(unittest.TestCase):

    def test_binary(self):
        representation_1 = BinaryRepresentation()
        representation_2 = BinaryRepresentation()

        self.assertNotEqual(representation_1.genome, representation_2.genome)

        compatibility = representation_1.compatibility(representation_2)
        self.assertNotEqual(compatibility, 0.0)

    def test_integer(self):
        representation_1 = IntegerRepresentation()
        representation_2 = IntegerRepresentation()

        self.assertNotEqual(representation_1.genome, representation_2.genome)

        compatibility = representation_1.compatibility(representation_2)
        self.assertNotEqual(compatibility, 0.0)

    def test_real_valued(self):
        representation_1 = RealValuedRepresentation()
        representation_2 = RealValuedRepresentation()

        self.assertNotEqual(representation_1.genome, representation_2.genome)

        compatibility = representation_1.compatibility(representation_2)
        self.assertNotEqual(compatibility, 0.0)
