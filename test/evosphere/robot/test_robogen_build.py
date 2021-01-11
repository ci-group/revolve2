import unittest

from revolve.robot.body.robogen.robogen_genotype import IndirectRobogenGenotype, DirectRobogenGenotype
from revolve.robot.body.robogen.robogen_body import RobogenBodyBuilder, RobogenBody


class TestRobogenBuild(unittest.TestCase):

    def test_body_builder_indirect(self):
        builder = RobogenBodyBuilder()
        genotype = IndirectRobogenGenotype()

        body = builder.create(genotype)
        self.assertIsInstance(body, RobogenBody)
        self.assertTrue(len(body.modules) > 0)

    def test_body_builder_direct(self):
        builder = RobogenBodyBuilder()
        genotype = DirectRobogenGenotype()

        body = builder.create(genotype)
        self.assertIsInstance(body, RobogenBody)
        self.assertTrue(len(body.modules) > 0)

