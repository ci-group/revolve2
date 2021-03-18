import unittest

from revolve.robot.body.robogen.helper.robot_visualizer import generate_matrix, show
from test.evosphere.mock_body_builder import MockBodyBuilder
from test.evosphere.robot.mock_morphology import MockBody, MockBrain
from nca.core.actor.actors import Actors
from nca.core.actor.individual_factory import ActorFactory
from revolve.evosphere.ecosphere import Ecosphere
from revolve.robot.birth_clinic import RobotBirthClinic
from revolve.robot.body.robogen.robogen_genotype import IndirectRobogenGenotype, SelfOrganizingRobogenGenotype
from revolve.robot.body.robogen.robogen_body import RobogenBodyBuilder, RobogenBody
from revolve.robot.brain.brain import RobotBrain
from revolve.robot.brain.brain_builder import MockBrainBuilder
from revolve.robot.robot import Robot


class MockRobogenBirthClinic(RobotBirthClinic):

    def __init__(self):
        super().__init__(MockBrainBuilder(), RobogenBodyBuilder())


class MockBirthClinic(RobotBirthClinic):

    def __init__(self):
        super().__init__(MockBrainBuilder(), MockBodyBuilder())


class TestBirthClinic(unittest.TestCase):

    def test_create(self):
        clinic = MockRobogenBirthClinic()

        n = 10
        actors: Actors = ActorFactory(actor_type=Robot).create(n)
        for actor in actors:
            actor.genotype = SelfOrganizingRobogenGenotype()

        robots = clinic.create(actors, Ecosphere("test"))
        for robot in robots:
            body_matrix, connections, length, height = generate_matrix(robot.body.modules)
            show(body_matrix, connections, length, height)

        self.assertEqual(len(robots), n)

        for robot in robots:
            self.assertIsInstance(robot.body, RobogenBody)
            self.assertIsInstance(robot.brain, MockBrain)

    def test_mock(self):
        clinic = RobotBirthClinic()

        n = 10
        robots = clinic.create(ActorFactory(actor_type=Robot).create(n), Ecosphere("test"))

        self.assertEqual(len(robots), n)

        for robot in robots:
            self.assertIsInstance(robot.body, MockBody)
            self.assertIsInstance(robot.brain, MockBrain)

    def test_robogen(self):
        clinic = RobotBirthClinic(body_builder=RobogenBodyBuilder())
        genotype = IndirectRobogenGenotype()
        n = 10
        robots = clinic.create(ActorFactory(mapping=genotype, actor_type=Robot).create(n), ecosphere=Ecosphere("test"))

        self.assertEqual(len(robots), n)

        for robot in robots:
            self.assertIsInstance(robot.body, RobogenBody)
            self.assertIsInstance(robot.brain, RobotBrain)
