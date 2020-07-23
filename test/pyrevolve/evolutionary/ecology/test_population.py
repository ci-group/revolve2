import unittest
from typing import List

from pyrevolve.evolutionary import Individual
from pyrevolve.evolutionary.agents import TestAgents, Agents
from pyrevolve.evolutionary.ecology.population import Population


class TestPopulation(unittest.TestCase):

    def test_id(self):
        population1 = Population(TestAgents(n=3))
        population2 = Population(TestAgents(n=3))

        self.assertNotEqual(population1.id, population2.id)

    def test_generation(self):
        agents_start = TestAgents(n=3)
        agents_new = TestAgents(n=3)

        population = Population(agents_start)

        population.next_generation(agents_new)

        self.assertNotEqual(population.individuals, agents_start)
        self.assertEqual(population.individuals, agents_new)
        self.assertEqual(population.offspring, None)

    def test_improvement(self):
        agents1: Agents = TestAgents(n=3)

        agents2: Agents = TestAgents(n=3)
        for agent in agents2:
            agent.fitness.fitness = 1.0

        population1 = Population(agents1)
        population2 = Population(agents2)

        self.assertTrue(population1.did_improve(agents2))
        self.assertFalse(population2.did_improve(agents1))
