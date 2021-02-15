import unittest

from nca.core.actor.individual_factory import ActorFactory
from nca.core.ecology import PopulationEcology
from nca.core.ecology.speciation.genus_management import GenusManagement


class TestPopulationEcology(unittest.TestCase):
    n = 10

    def test_population(self):
        population_ecology: PopulationEcology = PopulationEcology()
        individual_factory: ActorFactory = ActorFactory()
        population_ecology.initialize(individual_factory.create(self.n))

        json_data = population_ecology.to_json()
        self.assertTrue(json_data['id'] >= 0)
        self.assertTrue(len(json_data['species']) == 1)

        population = json_data['species'][0]
        self.assertTrue(population['age'] >= 0)
        self.assertEqual(len(population['individuals']), self.n)
        self.assertEqual(len(population['fitness']), 2)

    def test_genus(self):
        population_ecology: PopulationEcology = PopulationEcology(population_management=GenusManagement())
        individual_factory: ActorFactory = ActorFactory()
        population_ecology.initialize(individual_factory.create(self.n))

        json_data = population_ecology.to_json()
        self.assertTrue(json_data['id'] >= 0)
        self.assertTrue(len(json_data['species']) > 0)

    def test_recover(self):
        population_ecology: PopulationEcology = PopulationEcology()
        individual_factory: ActorFactory = ActorFactory()
        population_ecology.initialize(individual_factory.create(self.n))

        population_ecology.export("test/test_recovery")

        recovered_ecology: PopulationEcology = PopulationEcology()
        recovered_ecology.restore("test/test_recovery")

        json_data = recovered_ecology.to_json()
        self.assertTrue(json_data['id'] >= 0)
        self.assertTrue(len(json_data['species']) > 0)
