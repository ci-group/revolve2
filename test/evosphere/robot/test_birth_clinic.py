import unittest

from evosphere.robot.mock_morphology import MockBody, MockBrain
from nca.core.actor.individual_factory import ActorFactory
from revolve.evosphere.ecosphere import Ecosphere
from revolve.robot.birth_clinic import RobotBirthClinic
from revolve.robot.body.body_builder import MockBodyBuilder
from revolve.robot.body.robogen_body import RobogenBodyBuilder, RobogenBody
from revolve.robot.brain.brain import RobotBrain
from revolve.robot.brain.brain_builder import MockBrainBuilder
from revolve.robot.robogen.robogen_genotype import IndirectRobogenGenotype
from revolve.robot.robot import Robot


class MockBirthClinic(RobotBirthClinic):

    def __init__(self):
        super().__init__(MockBrainBuilder(), MockBodyBuilder())


class TestBirthClinic(unittest.TestCase):

    def test_create(self):
        clinic = MockBirthClinic()

        n = 10
        robots = clinic.build(ActorFactory(actor_type=Robot).create(n), Ecosphere("test"))

        self.assertEqual(len(robots), n)

        for robot in robots:
            self.assertIsInstance(robot.body, MockBody)
            self.assertIsInstance(robot.brain, MockBrain)

    def test_robogen(self):
        clinic = RobotBirthClinic(body_builder=RobogenBodyBuilder())
        genotype = IndirectRobogenGenotype()
        n = 10
        robots = clinic.build(ActorFactory(mapping=genotype, actor_type=Robot).create(n), ecosphere=Ecosphere("test"))

        self.assertEqual(len(robots), n)

        for robot in robots:
            self.assertIsInstance(robot.body, RobogenBody)
            self.assertIsInstance(robot.brain, RobotBrain)
