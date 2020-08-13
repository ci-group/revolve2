import copy
import unittest

from nca.core.abstract.composite.tree_helper import Orientation
from revolve.robot.body.robogen.robogen_grammar import RobogenSymbol
from revolve.robot.body.robogen.robogen_operators import RobogenInitialization, RobogenSwapMutation, RobogenInversionMutation, RobogenInsertMutation, RobogenReplaceMutation
from revolve.robot.body.robogen.robogen_representation import RobogenRepresentation


class TestRobogenManipulation(unittest.TestCase):

    def test_swap_mutation(self):
        mutation = RobogenSwapMutation()

        representation = RobogenRepresentation()
        representation._add(representation.core, RobogenSymbol.BLOCK, Orientation.LEFT)
        representation._add(representation.core, RobogenSymbol.BLOCK, Orientation.RIGHT)

        new_representation = copy.deepcopy(representation)
        mutation._execute(new_representation, RobogenInitialization())

        self.assertTrue(len(representation.genome) > 0)
        self.assertNotEqual(representation.genome, new_representation.genome)

        for index in range(len(representation.genome)):
            self.assertEqual(representation.genome[index].coordinate, new_representation.genome[index].coordinate)
            self.assertNotEqual(representation.genome[index].id, new_representation.genome[index].id)

        self.assertTrue(new_representation.is_valid())

    def test_inversion_mutation(self):
        mutation = RobogenInversionMutation()

        representation = RobogenRepresentation()
        representation._add(representation.core, RobogenSymbol.BLOCK, Orientation.LEFT)
        representation._add(representation.core, RobogenSymbol.BLOCK, Orientation.RIGHT)

        new_representation = copy.deepcopy(representation)
        mutation._execute(new_representation, RobogenInitialization())

        self.assertTrue(len(representation.genome) > 0)
        self.assertNotEqual(representation.genome, new_representation.genome)

        for index in range(len(representation.genome)):
            self.assertEqual(representation.genome[index].coordinate, new_representation.genome[index].coordinate)
            self.assertNotEqual(representation.genome[index].id, new_representation.genome[index].id)

        self.assertTrue(new_representation.is_valid())

    """
    def test_insert_mutation(self):
        mutation = RobogenInsertMutation()

        representation = RobogenRepresentation()
        representation._add(representation.core, RobogenSymbol.BLOCK, Orientation.LEFT)
        representation._add(representation.core, RobogenSymbol.BLOCK, Orientation.RIGHT)

        new_representation = copy.deepcopy(representation)
        mutation._execute(new_representation, RobogenInitialization())

        self.assertTrue(len(representation.genome) > 0)
        self.assertTrue(len(new_representation.genome) > len(representation.genome))

        print(new_representation.genome, representation.genome)

        self.assertTrue(new_representation.is_valid())
    """

    def test_replace_mutation(self):
        mutation = RobogenReplaceMutation()

        representation = RobogenRepresentation()
        representation._add(representation.core, RobogenSymbol.BLOCK, Orientation.LEFT)
        representation._add(representation.core, RobogenSymbol.BLOCK, Orientation.RIGHT)

        new_representation = copy.deepcopy(representation)
        mutation._execute(new_representation, RobogenInitialization())

        self.assertTrue(len(representation.genome) > 0)
        self.assertTrue(len(new_representation.genome) == len(representation.genome))
        self.assertNotEqual(representation.genome, new_representation.genome)

        for index in range(len(representation.genome)):
            self.assertEqual(representation.genome[index].coordinate, new_representation.genome[index].coordinate)

        self.assertTrue(new_representation.is_valid())