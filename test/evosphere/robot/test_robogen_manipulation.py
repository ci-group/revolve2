import copy
import unittest

from nca.core.abstract.structural.tree.tree_helper import Orientation
from nca.core.genome.operators.mutation_operator import SwapMutation, ReplaceMutation, InversionMutation, InsertMutation
from revolve.robot.robogen.robogen_grammar import RobogenSymbol

from revolve.robot.robogen.robogen_representation import RobogenRepresentation

class TestRobogenManipulation(unittest.TestCase):

    def test_swap_mutation(self):
        mutation = SwapMutation()

        representation = RobogenRepresentation()
        representation.genome = []
        representation._add(representation.core, RobogenSymbol.BLOCK, Orientation.LEFT)
        representation._add(representation.core, RobogenSymbol.BLOCK, Orientation.RIGHT)

        new_representation = copy.deepcopy(representation)
        mutation._mutate(new_representation)

        self.assertTrue(len(representation.genome) > 0)
        self.assertNotEqual(representation.genome, new_representation.genome)

        for index in range(len(representation.genome)):
            self.assertEqual(representation[index].coordinate, new_representation[index].coordinate)
            self.assertNotEqual(representation[index].id, new_representation[index].id)

        self.assertTrue(new_representation.is_valid())

    def test_inversion_mutation(self):
        mutation = InversionMutation()

        representation = RobogenRepresentation()
        representation.genome = []
        representation._add(representation.core, RobogenSymbol.BLOCK, Orientation.LEFT)
        representation._add(representation.core, RobogenSymbol.BLOCK, Orientation.RIGHT)

        new_representation = copy.deepcopy(representation)
        mutation._mutate(new_representation)

        self.assertTrue(len(representation.genome) > 0)
        self.assertNotEqual(representation.genome, new_representation.genome)

        for index in range(len(representation.genome)):
            self.assertEqual(representation[index].coordinate, new_representation[index].coordinate)
            self.assertNotEqual(representation[index].id, new_representation[index].id)

        self.assertTrue(new_representation.is_valid())

    def test_insert_mutation(self):
        mutation = InsertMutation()

        representation = RobogenRepresentation()
        representation.genome = []
        representation._add(representation.core, RobogenSymbol.BLOCK, Orientation.LEFT)
        representation._add(representation.core, RobogenSymbol.BLOCK, Orientation.RIGHT)

        new_representation = copy.deepcopy(representation)
        mutation._mutate(new_representation)

        self.assertTrue(len(representation.genome) > 0)
        self.assertTrue(len(new_representation.genome) > len(representation.genome))
        self.assertTrue(new_representation.is_valid())

    def test_replace_mutation(self):
        mutation = ReplaceMutation()

        representation = RobogenRepresentation()
        representation.genome = []
        representation._add(representation.core, RobogenSymbol.BLOCK, Orientation.LEFT)
        representation._add(representation.core, RobogenSymbol.BLOCK, Orientation.RIGHT)

        new_representation = copy.deepcopy(representation)
        mutation._mutate(new_representation)

        self.assertTrue(len(representation.genome) > 0)
        self.assertTrue(len(new_representation.genome) == len(representation.genome))
        self.assertNotEqual(representation.genome, new_representation.genome)

        #for index in range(len(representation.genome)):
        #    self.assertEqual(representation[index].coordinate, new_representation[index].coordinate)

        self.assertTrue(new_representation.is_valid())
