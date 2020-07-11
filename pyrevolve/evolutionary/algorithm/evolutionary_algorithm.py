from pyrevolve.evolutionary.algorithm.ecology.population_ecology import PopulationEcology
from pyrevolve.evolutionary.algorithm.evolutionary_configurations import EvolutionaryConfiguration


class EvolutionaryAlgorithm:

    def __init__(self, configuration: EvolutionaryConfiguration, population_ecology: PopulationEcology):
        self.configuration: EvolutionaryConfiguration = configuration
        self.population_ecology: PopulationEcology = population_ecology

        self.parent_selection = self.configuration.parent_selection
        self.survivor_selection = self.configuration.survivor_selection

        self.recombination = self.configuration.recombination
        self.mutation = self.configuration.mutation

        self.initialisation = self.configuration.initialisation
        self.termination_condition = self.configuration.termination_condition
        self.special_features = self.configuration.special_features

    def run(self):

        self.population_ecology.load()

        for _ in range(self.configuration.number_of_generations):

            agents: Agents = self.birth_clinic.create_agents()

            for environment in self.environments:
                self.population_ecology.create(agents, environment)
                for population in self.population_ecology.populations():
                    environment.population = population
                    self.simulator.evaluate(environment)

            self.population_ecology.select()

            #TODO evolve
            self.population_ecology.reproduce()

            self.population_ecology.evolve()

            self.population_ecology.export()

    def select_parents(self):
        self.population_ecology.select(self.parent_selection.algorithm)

    def select_survivors(self):
        self.population_ecology.select(self.survivor_selection.algorithm)

    def recombine_offspring(self):
        self.population_ecology.variate(self.recombination.algorithm)

    def mutate_offspring(self):
        self.population_ecology.variate(self.mutation.algorithm)