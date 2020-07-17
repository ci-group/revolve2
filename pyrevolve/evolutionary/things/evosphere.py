from typing import List

from pyrevolve.developmental.developmental_learning import DevelopmentalLearner
from pyrevolve.evolutionary.agents import Agents
from pyrevolve.evolutionary.algorithm.ecology.population_ecology import PopulationEcology
from pyrevolve.evolutionary.algorithm.ecology.population_management import PopulationManagement
from pyrevolve.evolutionary.algorithm.evolutionary_algorithm import EvolutionaryAlgorithm
from pyrevolve.evolutionary.algorithm.evolutionary_configurations import GeneticAlgorithmConfiguration
from pyrevolve.evolutionary.algorithm.genome.representation import Representation
from pyrevolve.evolutionary.algorithm.selection.selection import Selection
from pyrevolve.evolutionary.robotics.birth_clinic import BirthClinic
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
        self.developmental_leaner: DevelopmentalLearner = developmental_learner

        self.environments: List[Environment] = environments

        self.simulator = simulator


    def evolve(self):
        # load and initialize
        self.population_ecology.load()
        self.evolutionary_algorithm.initialize(self.population_ecology)

        agents: Agents = self.birth_clinic.create_robots()
        self.population_ecology.create(agents)  # , self.environments)

        # Run through iterations
        for _ in range(self.configuration.number_of_generations):

            for environment in self.environments:
                for population in self.population_ecology.populations():
                    environment.agents = population
                    self.simulator.evaluate(environment)

            done = self.evolutionary_algorithm.run(self.population_ecology)

            if done:
                break

            for environment in self.environments:
                for offspring in self.population_ecology.offsprings():
                    environment.agents = offspring
                    self.simulator.evaluate(environment)

            self.population_ecology.export()


class DefaultEvoSphere(EvoSphere):

    def __init__(self,
                 birth_clinic: BirthClinic = BirthClinic(Representation(), Representation()),
                 population_ecology: PopulationEcology = PopulationEcology(PopulationManagement(Selection())),
                 evolutionary_algorithm: EvolutionaryAlgorithm = EvolutionaryAlgorithm(GeneticAlgorithmConfiguration()),
                 developmental_learner: DevelopmentalLearner = DevelopmentalLearner(),
                 environments: List[Environment] = [],
                 simulator: Simulator = Simulator()):
        super().__init__(birth_clinic, population_ecology, evolutionary_algorithm, developmental_learner, environments, simulator)
