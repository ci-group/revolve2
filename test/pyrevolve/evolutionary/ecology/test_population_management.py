import unittest

from pyrevolve.evolutionary.agents import Agents
from pyrevolve.evolutionary.ecology.population_management import PopulationManagement
from pyrevolve.evolutionary.individual_factory import IndividualFactory


class PopulationManagementTest(unittest.TestCase):

    def test_management(self):
        population_management = PopulationManagement()

        agents1: Agents = IndividualFactory().create(n=3)
        population_management.initialize(agents1)

        self.assertEqual(population_management.population.individuals, agents1)

        agents2: Agents = IndividualFactory().create(n=3)
        population_management.initialize(agents2)

        self.assertEqual(population_management.population.individuals, agents2)
