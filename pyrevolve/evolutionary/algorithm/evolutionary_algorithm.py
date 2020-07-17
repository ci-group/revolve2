
from pyrevolve.evolutionary.algorithm.ecology.population_ecology import PopulationEcology
from pyrevolve.evolutionary.algorithm.evolutionary_configurations import EvolutionaryConfiguration
from pyrevolve.evolutionary.algorithm.genome.operators.mutation.mutation_operator import MutationOperator
from pyrevolve.evolutionary.algorithm.genome.operators.recombination.recombination_operator import RecombinationOperator
from pyrevolve.evolutionary.algorithm.initialisation import Initialisation
from pyrevolve.evolutionary.algorithm.selection.selection import ParentSelection, SurvivorSelection
from pyrevolve.evolutionary.algorithm.special_features import SpecialFeatures
from pyrevolve.evolutionary.algorithm.termination_condition import TerminationCondition


class EvolutionaryAlgorithm:

    def __init__(self, configuration: EvolutionaryConfiguration):
        self.configuration: EvolutionaryConfiguration = configuration

        self.parent_selection: ParentSelection              = self.configuration.parent_selection
        self.survivor_selection: SurvivorSelection          = self.configuration.survivor_selection

        self.recombination: RecombinationOperator           = self.configuration.recombination
        self.mutation: MutationOperator                     = self.configuration.mutation

        self.initialisation: Initialisation                 = self.configuration.initialisation
        self.termination_condition: TerminationCondition    = self.configuration.termination_condition
        self.special_features: SpecialFeatures              = self.configuration.special_features

    def initialize(self, population):
        self.initialisation.algorithm(population)

    def run(self, population_ecology: PopulationEcology):

        if self.termination_condition:
            return False

        for population in population_ecology.populations():

            self.parent_selection.algorithm(population)

            self.recombination.algorithm(population)

            self.mutation.algorithm(population)

        return True

    def survival(self, population_ecology: PopulationEcology):
        for index, population in enumerate(population_ecology.populations()):
            self.survivor_selection.algorithm(population)

