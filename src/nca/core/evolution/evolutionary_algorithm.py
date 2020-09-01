from typing import List

from nca.core.actor.actors import Actors
from nca.core.actor.individual import Individual
from nca.core.ecology.population import Population
from nca.core.evolution.conditions.initialization import Initialization
from nca.core.evolution.conditions.special_features import SpecialFeatures
from nca.core.evolution.conditions.condition import Condition
from nca.core.evolution.evolutionary_configurations import EvolutionConfiguration, GeneticAlgorithmConfiguration
from nca.core.evolution.selection.selection import ParentSelection, SurvivorSelection
from nca.core.genome.operators.mutation_operator import MutationOperator
from nca.core.genome.operators.recombination_operator import RecombinationOperator


class EvolutionaryAlgorithm:

    def __init__(self, configuration: EvolutionConfiguration = GeneticAlgorithmConfiguration()):
        self.configuration: EvolutionConfiguration = configuration

        self.parent_selection: ParentSelection = self.configuration.parent_selection
        self.survivor_selection: SurvivorSelection = self.configuration.survivor_selection

        self.recombination: RecombinationOperator = self.configuration.recombination
        self.mutation: MutationOperator = self.configuration.mutation

        self.initialization_type: type(Initialization) = self.configuration.initialization_type
        self.termination_condition: Condition = self.configuration.condition
        self.special_features: SpecialFeatures = self.configuration.special_features

    def should_terminate(self, population: Population):
        return self.termination_condition.terminate(population)

    def run(self, population: Population, evaluator):
        parents_list: List[Actors] = self.parent_selection.select(population.individuals)

        offspring: Actors = Actors([Individual(self.mutation(self.recombination(parents))) for parents in parents_list])

        evaluator(offspring)

        population.individuals.extend(offspring)
        population.next_generation(self.survivor_selection.select(population.individuals))
