import math
import unittest

from nca.core.agent.agents import Agents
from nca.core.agent.individual_factory import IndividualFactory
from nca_test.core.representation.mock_representation import MockRepresentation


class TestAgents(unittest.TestCase):

    def test_compare(self):
        n = 3
        agents: Agents = IndividualFactory(MockRepresentation).initialize().create(n)

        self.assertEqual(len(agents), n)

        for agent in agents:
            self.assertIsInstance(agent.representation, MockRepresentation)
            self.assertTrue(agent.fitness == 0.0)

    def test_id(self):
        agents: Agents = IndividualFactory(MockRepresentation).initialize().create(2)

        self.assertNotEqual(agents[0].id, agents[1].id)

    def test_age(self):
        n = 3
        agents = IndividualFactory().initialize().create(n)

        for index in range(n):
            agents.update_age()
            for agent in agents:
                self.assertEqual(agent.age.generations, index+1)
