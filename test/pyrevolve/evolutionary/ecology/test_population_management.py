import unittest
from typing import List

from pyrevolve.evolutionary import Individual
from pyrevolve.evolutionary.agents import TestAgents, Agents
from pyrevolve.evolutionary.ecology.population import Population
from pyrevolve.evolutionary.ecology.population_management import TestPopulationManagement


class PopulationManagementTest(unittest.TestCase):

    def test_management(self):
        population_management = TestPopulationManagement()

        agents1: Agents = TestAgents(n=3)
        population_management.create_population(agents1)

        self.assertEqual(population_management.population.individuals, agents1)

        agents2: Agents = TestAgents(n=3)
        population_management.create_population(agents2)

        self.assertEqual(population_management.population.individuals, agents2)
