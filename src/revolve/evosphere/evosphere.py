from typing import List

from nca.core.abstract.configurations import EvoSphereConfiguration
from nca.core.agent.agents import Agents
from nca.core.agent.fitness import Fitness
from nca.core.ecology import PopulationEcology
from nca.core.ecology.population import Population
from nca.core.ecology.population_management import PopulationManagement
from nca.core.evolution.evolutionary_algorithm import EvolutionaryAlgorithm
from nca.core.genome.grammar.lsystem_representation import LSystemRepresentation
from revolve.evosphere.configuration import GeneticAlgorithmConfiguration
from revolve.evosphere.environment import Environment
from revolve.robot.birth_clinic import BirthClinic
from revolve.robot.body.body_builder import BodyBuilder
from revolve.robot.brain.brain_builder import BrainBuilder
from revolve.robot.brain.representation.multineat_representation import MultiNEATRepresentation
from revolve.tol.developmental_learning import DevelopmentalLearner
from simulation.simulation_manager import SimulationManager
from simulation.simulator.simulator_helper import SimulatorType, RequestCommand


class EvoSphere:

    def __init__(self,
                 birth_clinic: BirthClinic,
                 population_ecology: PopulationEcology,
                 evolutionary_algorithm: EvolutionaryAlgorithm,
                 developmental_learner: DevelopmentalLearner,
                 environments: List[Environment],
                 simulation: SimulationManager):
        self.configuration = EvoSphereConfiguration()

        self.birth_clinic: BirthClinic = birth_clinic

        self.population_ecology: PopulationEcology = population_ecology

        self.evolutionary_algorithm: EvolutionaryAlgorithm = evolutionary_algorithm
        self.developmental_learner: DevelopmentalLearner = developmental_learner

        self.environments: List[Environment] = environments

        self.simulation = simulation

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
            self.simulation.simulate(environment.agents, RequestCommand(environment, SimulatorType.NONE))


class DefaultEvoSphere(EvoSphere):

    def __init__(self,
                 birth_clinic: BirthClinic = BirthClinic(BodyBuilder(LSystemRepresentation),
                                                         BrainBuilder(MultiNEATRepresentation), Fitness),
                 population_ecology: PopulationEcology = PopulationEcology(PopulationManagement()),
                 evolutionary_algorithm: EvolutionaryAlgorithm = EvolutionaryAlgorithm(GeneticAlgorithmConfiguration(),
                                                                                       Fitness),
                 developmental_learner: DevelopmentalLearner = DevelopmentalLearner(),
                 environments: List[Environment] = None,
                 simulation: SimulationManager = SimulationManager()):
        super().__init__(birth_clinic, population_ecology, evolutionary_algorithm,
                         developmental_learner, environments, simulation)
