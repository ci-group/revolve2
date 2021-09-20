import unittest

from revolve2.revolve.robot.body.robogen.robogen_genotype import IndirectRobogenGenotype, DirectRobogenGenotype
from revolve2.revolve.robot.body.robogen.robogen_body import RobogenBodyBuilder, RobogenBody
from revolve2.revolve.robot.development_request import BodyDevelopmentRequest


class TestRobogenBuild(unittest.TestCase):

    def test_body_builder_indirect(self):
        builder = RobogenBodyBuilder()
        genotype = IndirectRobogenGenotype()
        body_development_request = BodyDevelopmentRequest(0, genotype, None)
        body = builder.create(body_development_request)
        self.assertIsInstance(body, RobogenBody)
        self.assertTrue(len(body.modules) > 0)

    def test_body_builder_direct(self):
        builder = RobogenBodyBuilder()
        genotype = DirectRobogenGenotype()
        body_development_request = BodyDevelopmentRequest(0, genotype, None)
        body = builder.create(body_development_request)
        body.visualize()
        print(body.morphological_measures)
        self.assertIsInstance(body, RobogenBody)
        self.assertTrue(len(body.modules) > 0)

