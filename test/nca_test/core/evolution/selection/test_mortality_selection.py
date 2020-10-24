import unittest

from nca.core.abstract.configurations import PopulationConfiguration
from nca.core.actor.individual import Individual
from nca.core.actor.individual_factory import ActorFactory
from nca.core.ecology.population import Population
from nca.core.evolution.selection.mortality_selection import OffspringMortalitySelection, PopulationMortalitySelection, \
    ConstantMortalitySelection, NullMortalitySelection
from nca.core.evolution.selection.non_dominated_survival import NonDominatedSortingSurvival
from nca.core.evolution.selection.survivor_selection import NullSurvivorSelection, FitnessSteadyStateSelection, \
    GenerationalSteadyStateSelection, ElitismSelection


class TestMortalitySelection(unittest.TestCase):

    def test_null_selection(self):
        population = Population(ActorFactory().create(PopulationConfiguration().population_size))
        offspring = ActorFactory().create(int(PopulationConfiguration().population_size / 2))

        mortality_selection = NullMortalitySelection()
        selected = mortality_selection(population.individuals, offspring)
        for element in selected:
            self.assertIsInstance(element, Individual)

        # TODO population individuals is overriden
        self.assertTrue(len(selected) == len(population.individuals))
        self.assertTrue(len(selected) >= mortality_selection.configuration.population_size)

    def test_offspring_selection(self):
        population = Population(ActorFactory().create(PopulationConfiguration().population_size))
        offspring = ActorFactory().create(int(PopulationConfiguration().population_size / 2))

        mortality_selection = OffspringMortalitySelection()
        selected = mortality_selection(population.individuals, offspring)
        for element in selected:
            self.assertIsInstance(element, Individual)

        self.assertTrue(len(selected) <= len(population.individuals) + len(offspring))
        self.assertTrue(len(selected) >= mortality_selection.configuration.population_size)

    def test_population_selection(self):
        population = Population(ActorFactory().create(PopulationConfiguration().population_size))
        offspring = ActorFactory().create(int(PopulationConfiguration().population_size / 2))

        mortality_selection = PopulationMortalitySelection()
        selected = mortality_selection(population.individuals, offspring)
        for element in selected:
            self.assertIsInstance(element, Individual)

        self.assertTrue(len(selected) <= len(population.individuals) + len(offspring))
        self.assertTrue(len(selected) >= mortality_selection.configuration.population_size)

    def test_constant_selection(self):
        population = Population(ActorFactory().create(PopulationConfiguration().population_size))
        offspring = ActorFactory().create(int(PopulationConfiguration().population_size / 2))

        mortality_selection = ConstantMortalitySelection()
        selected = mortality_selection(population.individuals, offspring)
        for element in selected:
            self.assertIsInstance(element, Individual)

        self.assertTrue(len(selected) <= len(population.individuals) + len(offspring))
        self.assertTrue(len(selected) >= mortality_selection.configuration.population_size)
