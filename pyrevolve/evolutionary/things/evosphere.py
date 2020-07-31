from typing import List

from pyrevolve.developmental.developmental_learning import DevelopmentalLearner
from pyrevolve.evolutionary import Fitness
from pyrevolve.evolutionary.agents import Agents
from pyrevolve.evolutionary.algorithm.genome.representations.l_system.lsystem_representation import \
    LSystemRepresentation
from pyrevolve.evolutionary.ecology.population import Population
from pyrevolve.evolutionary.ecology.population_ecology import PopulationEcology
from pyrevolve.evolutionary.algorithm.evolutionary_algorithm import EvolutionaryAlgorithm
from pyrevolve.evolutionary.algorithm.evolutionary_configurations import GeneticAlgorithmConfiguration
from pyrevolve.evolutionary.ecology.population_management import PopulationManagement
from pyrevolve.evolutionary.robotics.birth_clinic import BirthClinic
from pyrevolve.evolutionary.robotics.morphology.body.body_builder import BodyBuilder
from pyrevolve.evolutionary.robotics.morphology.brain.brain_builder import BrainBuilder
from pyrevolve.evolutionary.robotics.morphology.brain.representation.multineat_representation import \
    MultiNEATRepresentation
from pyrevolve.evolutionary.things.environment import Environment
from pyrevolve.shared.configurations import EvoSphereConfiguration

from pyrevolve.simulator.simulator import Simulator


class EvoSphere:

    def __init__(self,
                 birth_clinic: BirthClinic,
                 population_ecology: PopulationEcology,
                 evolutionary_algorithm: EvolutionaryAlgorithm,
                 developmental_learner: DevelopmentalLearner,
                 environments: List[Environment],
                 simulator: Simulator):
        self.configuration = EvoSphereConfiguration()

        self.birth_clinic: BirthClinic = birth_clinic

        self.population_ecology: PopulationEcology = population_ecology

        self.evolutionary_algorithm: EvolutionaryAlgorithm = evolutionary_algorithm
        self.developmental_learner: DevelopmentalLearner = developmental_learner

        self.environments: List[Environment] = environments

        self.simulator = simulator

    def evolve(self):
        # load and initialize
        self.population_ecology.load()
        self.evolutionary_algorithm.initialize(self.population_ecology.populations())

        robots: Agents = self.birth_clinic.create()
        self.population_ecology.initialize(robots)  # , self.environments)

        print("Evaluate population")

        for population in self.population_ecology.populations():
            self.evaluate(population)

        # Run through iterations
        for generation_index in range(self.configuration.number_of_generations):
            print("Generation ", generation_index)
            for population in self.population_ecology.populations():
                if self.evolutionary_algorithm.should_terminate(population):
                    print("Terminated evolution due to termination condition.")
                    break

                self.evolutionary_algorithm.run(population, self.evaluate)

            self.population_ecology.export()

            self.population_ecology.speciate()

    def evaluate(self, population: Population):
        for environment in self.environments:
            environment.agents = population.individuals
            self.simulator.evaluate(environment)


class DefaultEvoSphere(EvoSphere):

    def __init__(self,
                 birth_clinic: BirthClinic = BirthClinic(BodyBuilder(LSystemRepresentation),
                                                         BrainBuilder(MultiNEATRepresentation), Fitness),
                 population_ecology: PopulationEcology = PopulationEcology(PopulationManagement()),
                 evolutionary_algorithm: EvolutionaryAlgorithm = EvolutionaryAlgorithm(GeneticAlgorithmConfiguration(),
                                                                                       Fitness),
                 developmental_learner: DevelopmentalLearner = DevelopmentalLearner(),
                 environments: List[Environment] = None,
                 simulator: Simulator = Simulator()):
        super().__init__(birth_clinic, population_ecology, evolutionary_algorithm,
                         developmental_learner, environments, simulator)
