import unittest

from revolve.robot.birth_clinic import RobotFactory
from revolve.robot.morphology import MorphologyType
from simulation_test.test_simulation_manager import MockBodyRepresentation, MockBrainRepresentation


class TestRobotFactory(unittest.TestCase):

    def test_create(self):
        factory = RobotFactory(MockBodyRepresentation, MockBrainRepresentation)
        n = 10
        robots = factory.create(n)

        self.assertEqual(len(robots), n)

        for index in range(n):
            self.assertIsInstance(robots[index].representation[MorphologyType.BODY].genome, MockBodyRepresentation)
            self.assertIsInstance(robots[index].representation[MorphologyType.BRAIN].genome, MockBrainRepresentation)
