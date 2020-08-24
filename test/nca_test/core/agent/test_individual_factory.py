import unittest

from nca.core.agent.agents import Agents
from nca.core.agent.individual_factory import IndividualFactory, AgentFactory


class TestIndividualFactory(unittest.TestCase):

    def test_individual(self):
        n = 5
        factory = IndividualFactory()
        factory.initialize()

        individuals = factory.create(n)

        self.assertIsInstance(individuals, Agents)
        self.assertTrue(len(individuals), n)
        self.assertNotEqual(individuals[0].representation, individuals[1].representation)
        self.assertEqual(individuals[0].fitness, individuals[1].fitness)

    # TODO wait for Brain Representation to emerge.
    """def test_agent(self): 
        n = 5
        factory = AgentFactory()
        factory.initialize()
        agents = factory.create(n)

        self.assertIsInstance(agents, Agents)
        self.assertTrue(len(agents), n)
        self.assertNotEqual(agents[0].representation, agents[1].representation)
        self.assertEqual(agents[0].fitness, agents[1].fitness)
    """