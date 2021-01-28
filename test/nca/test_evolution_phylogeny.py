import pickle
import unittest

from nca.core.ecology import PopulationEcology
from nca.core.ecology.metrics import PhylogenyMetrics
from nca.evolution import Evolution


class TestPhylogeny(unittest.TestCase):
    file_name = 'test_evolution.pkl'

    def test_phylogeny(self):
        evolution = Evolution()
        population_ecology = evolution.evolve()
        with open(self.file_name, 'wb') as output:
            pickle.dump(population_ecology, output, pickle.HIGHEST_PROTOCOL)

    def __initialize(self):
        with open(self.file_name, 'rb') as input:
            population_ecology: PopulationEcology = pickle.load(input)

        phylogeny = PhylogenyMetrics(population_ecology.management.population.history)
        return phylogeny, population_ecology

    def test_statistics(self):
        phylogeny, population_ecology = self.__initialize()

        phylogeny.generational_statistics()

    def test_depth(self):
        phylogeny, population_ecology = self.__initialize()
        # print("total history len: ", len(phylogeny.complete_history.keys()))
        for individual in phylogeny.complete_history.values():
            depth = phylogeny.generational_depth(individual, max_depth=3)
            #print("id ", individual.id)
            #for key in depth.keys():
            #    print("parent ", str(key), ": depth links ", str(depth[key]))

    def test_graph(self):
        phylogeny, population_ecology = self.__initialize()
        phylogeny.graph()

    def test_heritability(self):
        phylogeny, population_ecology = self.__initialize()
        phylogeny.heritability(50)



