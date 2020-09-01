from nca.core.abstract.configurations import EvoSphereConfiguration
from nca.core.actor.actors import Actors
from nca.core.actor.fitness import Fitness
from nca.core.actor.fitnesses import OnesFitness
from nca.core.actor.individual_factory import IndividualFactory
from nca.core.ecology.population import Population
from nca.core.evolution.evolutionary_algorithm import EvolutionaryAlgorithm
from nca.core.evolution.evolutionary_configurations import GeneticAlgorithmConfiguration, EvolutionConfiguration


class Evolution:

    def __init__(self, evolutionary_configuration: EvolutionConfiguration = GeneticAlgorithmConfiguration(),
                 fitness: Fitness = OnesFitness(), individual_factory: IndividualFactory = IndividualFactory(),
                 debug=True):
        self.configuration = EvoSphereConfiguration()

        self.evolutionary_algorithm: EvolutionaryAlgorithm = EvolutionaryAlgorithm(evolutionary_configuration)
        self.fitness: Fitness = fitness

        self.debug = debug
        self.terminate = False

        self.population = Population(individual_factory.create(self.configuration.number_of_agents))

    def evolve(self):

        self.log("Evaluate population")

        self.evaluate(self.population.individuals)

        # Run through iterations
        for generation_index in range(self.configuration.number_of_generations):
            self.log("Generation " + str(generation_index))
            if self.evolutionary_algorithm.should_terminate(self.population):
                self.terminate = True
                break

            self.evolutionary_algorithm.run(self.population, self.evaluate)

            if self.terminate:
                self.log("Terminated evolution due to " + str(self.evolutionary_algorithm.termination_condition))
                break

        self.log(str(self.population))

    def evaluate(self, agents: Actors):
        for individual in agents:
            individual.performance(self.fitness)

    def log(self, string: str):
        if self.debug:
            print(string)

