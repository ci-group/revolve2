import unittest

import numpy as np

from nca.core.genome.representations.direct_representation import BinaryRepresentation, \
    RealValuedRepresentation, IntegerRepresentation


class DirectRepresentationTest(unittest.TestCase):
    """
    def test_same(self):
        representation_1 = BinaryRepresentation()
        representation_2 = BinaryRepresentation()

        representation_2.init(representation_1.genome)
        self.assertTrue(np.allclose(representation_1.genome, representation_2.genome))

        compatibility = representation_1.compatibility(representation_2)
        self.assertEqual(compatibility, 0.0)
    """
    
    def test_binary(self):
        representation_1 = BinaryRepresentation()
        representation_2 = BinaryRepresentation()

        self.assertFalse(np.allclose(representation_1.genome, representation_2.genome))

        compatibility = representation_1.compatibility(representation_2)
        self.assertNotEqual(compatibility, 0.0)

    def test_integer(self):
        representation_1 = IntegerRepresentation()
        representation_2 = IntegerRepresentation()

        self.assertFalse(np.allclose(representation_1.genome, representation_2.genome))

        compatibility = representation_1.compatibility(representation_2)
        self.assertNotEqual(compatibility, 0.0)

    def test_real_valued(self):
        representation_1 = RealValuedRepresentation()
        representation_2 = RealValuedRepresentation()

        self.assertFalse(np.allclose(representation_1.genome, representation_2.genome))

        compatibility = representation_1.compatibility(representation_2)
        self.assertNotEqual(compatibility, 0.0)
