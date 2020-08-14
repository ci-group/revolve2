import unittest

from nca.core.agent.agents import Agents
from nca.core.agent.individual_factory import IndividualFactory
from nca.core.ecology.population import Population


class TestPopulation(unittest.TestCase):

    def test_id(self):
        population1 = Population(IndividualFactory().create(3))
        population2 = Population(IndividualFactory().create(3))

        self.assertNotEqual(population1.id, population2.id)

    def test_generation(self):
        agents_start = IndividualFactory().create(3)
        agents_new = IndividualFactory().create(3)

        population = Population(agents_start)

        population.next_generation(agents_new)

        self.assertNotEqual(population.individuals, agents_start)
        self.assertEqual(population.individuals, agents_new)

    def test_improvement(self):
        agents1: Agents = IndividualFactory().create(3)

        agents2: Agents = IndividualFactory().create(3)
        for agent in agents2:
            agent.fitness = 1.0

        population1 = Population(agents1)
        population2 = Population(agents2)

        self.assertTrue(population1.did_improve(agents2))
        self.assertFalse(population2.did_improve(agents1))
