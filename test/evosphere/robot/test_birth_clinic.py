import unittest

from evosphere.robot.mock_morphology import MockBody, MockBrain
from nca.core.actor.individual import Individual
from nca.core.actor.individual_factory import IndividualFactory
from revolve.evosphere.ecosphere import Ecosphere
from revolve.robot.birth_clinic import BirthClinic, RobotBirthClinic
from revolve.robot.body.body_builder import BodyBuilder
from revolve.robot.brain.brain_builder import BrainBuilder
from revolve.robot.robot import Robot
#from simulation_test.test_simulation_manager import MockBrainBuilder, MockBodyBuilder


class MockBirthClinic(RobotBirthClinic):

    def __init__(self):
        super().__init__()
        #self.brain_builder: BrainBuilder = MockBrainBuilder()
        #self.body_builder: BodyBuilder = MockBodyBuilder()


class TestBirthClinic(unittest.TestCase):

    def test_create(self):
        clinic = MockBirthClinic()

        n = 10
        robots = clinic.build(IndividualFactory().create(n), Ecosphere("test"))

        self.assertEqual(len(robots), n)

        for robot in robots:
            self.assertIsInstance(robot.body, MockBody)
            self.assertIsInstance(robot.brain, MockBrain)
