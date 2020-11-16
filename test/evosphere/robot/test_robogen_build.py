import unittest

from revolve.robot.body.robogen_body import RobogenBodyBuilder, RobogenBody
from revolve.robot.robogen.robogen_genotype import IndirectRobogenGenotype, DirectRobogenGenotype


class TestRobogenBuild(unittest.TestCase):

    def test_body_builder_indirect(self):
        builder = RobogenBodyBuilder()
        genotype = IndirectRobogenGenotype()

        body = builder.build(genotype)
        self.assertIsInstance(body, RobogenBody)
        self.assertTrue(len(body.modules) > 0)

    def test_body_builder_direct(self):
        builder = RobogenBodyBuilder()
        genotype = DirectRobogenGenotype()

        body = builder.build(genotype)
        self.assertIsInstance(body, RobogenBody)
        self.assertTrue(len(body.modules) > 0)

