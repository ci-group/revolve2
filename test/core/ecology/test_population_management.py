import unittest

from nca.core.agent.agents import Agents
from nca.core.ecology.population_management import PopulationManagement
from nca.core.agent.individual_factory import IndividualFactory


class PopulationManagementTest(unittest.TestCase):

    def test_management(self):
        population_management = PopulationManagement()

        agents1: Agents = IndividualFactory().create(n=3)
        population_management.initialize(agents1)

        self.assertEqual(population_management.population.individuals, agents1)

        agents2: Agents = IndividualFactory().create(n=3)
        population_management.initialize(agents2)

        self.assertEqual(population_management.population.individuals, agents2)
