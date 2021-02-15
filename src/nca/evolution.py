from abstract.configurations import EvolutionConfiguration
from nca.core.actor.actors import Actors
from nca.core.actor.fitness_evaluation import OnesFitness, FitnessEvaluation
from nca.core.actor.individual_factory import ActorFactory
from nca.core.ecology import PopulationEcology
from nca.core.ecology.speciation.genus_management import GenusManagement
from nca.core.evolution.evolutionary_algorithm import EvolutionaryAlgorithm
from nca.core.evolution.evolutionary_configurations import GeneticAlgorithmConfiguration, EvolutionaryConfiguration


class Evolution:

    def __init__(self, evolutionary_configuration: EvolutionaryConfiguration = GeneticAlgorithmConfiguration(),
                 fitness_evaluation: FitnessEvaluation = OnesFitness(), individual_factory: ActorFactory = ActorFactory(),
                 debug=True):
        self.configuration = EvolutionConfiguration()

        self.evolutionary_algorithm: EvolutionaryAlgorithm = EvolutionaryAlgorithm(evolutionary_configuration)
        self.individual_factory: ActorFactory = individual_factory
        self.population_ecology: PopulationEcology = PopulationEcology()

        self.fitness_evaluation: FitnessEvaluation = fitness_evaluation

        self.debug: bool = debug
        self.terminate_run: bool = False

    def evolve(self):
        # INITIALISE population with random candidate solutions;
        self.population_ecology.initialize(self.individual_factory.create(self.configuration.number_of_agents))

        # EVALUATE each candidate;
        self.log("Evaluate population")
        for population in self.population_ecology.management.populations():
            self.evaluator(population.individuals)

        # REPEAT UNTIL (TERMINATION CONDITION is satisfied [or generations is reached]) DO
        for generation_index in range(self.configuration.number_of_generations):
            if generation_index % 25 == 0:
                self.log("Generation " + str(generation_index))

            terminated: bool = False

            for population_index, population in enumerate(self.population_ecology.management.populations()):
                # CHECK TERMINATION CONDITION is satisfied
                if self.evolutionary_algorithm.should_terminate(population):
                    terminated = True
                    self.log("Terminated evolution due to " + str(self.evolutionary_algorithm.termination_condition))
                    break

                # Select, Recombine, Mutate, Evaluate and Select new individuals
                self.evolutionary_algorithm.run(population, self.evaluator)

            if terminated:
                break

            self.population_ecology.process()

        return self.population_ecology

    def evaluator(self, agents: Actors):
        for individual in agents:
            individual.fitness.add(self.fitness_evaluation(individual))

    def log(self, string: str):
        if self.debug:
            print("log ", string)
