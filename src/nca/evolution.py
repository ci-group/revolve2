from nca.core.abstract.configurations import EvolutionConfiguration
from nca.core.actor.actors import Actors
from nca.core.actor.fitness import Fitness
from nca.core.actor.fitnesses import OnesFitness
from nca.core.actor.individual_factory import IndividualFactory
from nca.core.analysis.statistics import Statistics
from nca.core.ecology.population import Population
from nca.core.evolution.evolutionary_algorithm import EvolutionaryAlgorithm
from nca.core.evolution.evolutionary_configurations import GeneticAlgorithmConfiguration, EvolutionaryConfiguration


class Evolution:

    def __init__(self, evolutionary_configuration: EvolutionaryConfiguration = GeneticAlgorithmConfiguration(),
                 fitness: Fitness = OnesFitness(), individual_factory: IndividualFactory = IndividualFactory(),
                 debug=True):
        self.configuration = EvolutionConfiguration()

        self.evolutionary_algorithm: EvolutionaryAlgorithm = EvolutionaryAlgorithm(evolutionary_configuration)
        self.individual_factory: IndividualFactory = individual_factory

        self.fitness: Fitness = fitness

        self.debug: bool = debug
        self.terminate_run: bool = False

    def evolve(self):

        statistics = Statistics()

        # INITIALISE population with random candidate solutions;
        population = Population(self.individual_factory.create(self.configuration.number_of_agents))

        self.log("Evaluate population")
        # EVALUATE each candidate;
        self.evaluator(population.individuals)
        statistics.log(population)

        # REPEAT UNTIL (TERMINATION CONDITION is satisfied [or generations is reached]) DO
        for generation_index in range(self.configuration.number_of_generations):
            if generation_index % 25 == 0:
                self.log("Generation " + str(generation_index))

            # CHECK TERMINATION CONDITION is satisfied
            if self.evolutionary_algorithm.should_terminate(population):
                self.log("Terminated evolution due to " + str(self.evolutionary_algorithm.termination_condition))
                break

            # Select, Recombine, Mutate, Evaluate and Select new individuals
            self.evolutionary_algorithm.run(population, self.evaluator)
            statistics.log(population)

        return statistics

    def evaluator(self, agents: Actors):
        for individual in agents:
            individual.performance(self.fitness)

    def log(self, string: str):
        if self.debug:
            print(string)
