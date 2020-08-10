import unittest

from nca.core.agent.fitness import Fitness
from nca.core.genome.representation import Representation
from revolve.robot.birth_clinic import BirthClinic
from revolve.robot.body.body import Body
from revolve.robot.body.body_builder import BodyBuilder
from revolve.robot.brain.brain import Brain
from src.revolve.robot.brain.brain_builder import BrainBuilder


class TestBirthClinic(unittest.TestCase):

    def test_clinic(self):
        clinic = BirthClinic(BodyBuilder(Representation), BrainBuilder(Representation))

        robot = clinic.build_robot()

        self.assertIsInstance(robot.fitness, Fitness)
        self.assertIsInstance(robot.body, Body)
        self.assertIsInstance(robot.brain, Brain)

    def test_create(self):
        clinic = BirthClinic(BodyBuilder(Representation), BrainBuilder(Representation))
        n = 10
        robots = clinic.create(n)

        self.assertEqual(len(robots), n)

        for index in range(n):
            self.assertIsInstance(robots[index].fitness, Fitness)
            self.assertIsInstance(robots[index].body, Body)
            self.assertIsInstance(robots[index].brain, Brain)