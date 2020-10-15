from typing import List

from nca.core.actor.actors import Actors
from nca.core.actor.individual import Individual
from nca.core.analysis.statistics import Statistics
from nca.core.ecology.population import Population
from nca.core.evolution.conditions.initialization import Initialization
from nca.core.evolution.conditions.special_features import SpecialFeatures
from nca.core.evolution.conditions.condition import Condition
from nca.core.evolution.evolutionary_configurations import EvolutionaryConfiguration, GeneticAlgorithmConfiguration
from nca.core.evolution.selection.selection import ParentSelection, SurvivorSelection
from nca.core.genome.operators.mutation_operator import MutationOperator
from nca.core.genome.operators.recombination_operator import RecombinationOperator


class EvolutionaryAlgorithm:

    def __init__(self, configuration: EvolutionaryConfiguration = GeneticAlgorithmConfiguration()):
        self.configuration: EvolutionaryConfiguration = configuration

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
        # (PARENT) SELECT parents
        parents_list: List[Actors] = self.parent_selection.select(population.individuals)

        # RECOMBINE, then MUTATE resulting recombination to create resulting offspring
        offspring: Actors = Actors([Individual(self.mutation(self.recombination(parents))) for parents in parents_list])
        population.individuals.extend(offspring)

        # EVALUATE the candidates
        evaluator(offspring)

        # (SURVIVOR) SELECT individuals for the next generation
        population.next_generation(self.survivor_selection.select(population.individuals))
