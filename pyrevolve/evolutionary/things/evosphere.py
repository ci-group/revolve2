from typing import List

from pyrevolve.developmental.developmental_learning import DevelopmentalLearner
from pyrevolve.evolutionary.agents import Agents
from pyrevolve.evolutionary.ecology.population import Population
from pyrevolve.evolutionary.ecology.population_ecology import PopulationEcology
from pyrevolve.evolutionary.ecology import PopulationManagement
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
        self.developmental_learner: DevelopmentalLearner = developmental_learner

        self.environments: List[Environment] = environments

        self.simulator = simulator

    def evolve(self):
        # load and initialize
        self.population_ecology.load()
        self.evolutionary_algorithm.initialize(self.population_ecology.populations())

        agents: Agents = self.birth_clinic.create_robots()
        self.population_ecology.initialize(agents)  # , self.environments)

        print("Evaluate population")

        self.evaluate(self.population_ecology.populations())

        # Run through iterations
        for generation_index in range(self.configuration.number_of_generations):
            print("Generation ", generation_index)

            if self.evolutionary_algorithm.should_terminate(self.population_ecology.populations()):
                break

            self.evolutionary_algorithm.run(self.population_ecology.populations(), self.evaluate)

            self.population_ecology.export()

    def evaluate(self, agents: List[Population]):
        for environment in self.environments:
            environment.agents = agents
            self.simulator.evaluate(environment)


class DefaultEvoSphere(EvoSphere):

    def __init__(self,
                 birth_clinic: BirthClinic = BirthClinic(Representation(), Representation()),
                 population_ecology: PopulationEcology = PopulationEcology(PopulationManagement(Selection())),
                 evolutionary_algorithm: EvolutionaryAlgorithm = EvolutionaryAlgorithm(GeneticAlgorithmConfiguration()),
                 developmental_learner: DevelopmentalLearner = DevelopmentalLearner(),
                 environments: List[Environment] = [],
                 simulator: Simulator = Simulator()):
        super().__init__(birth_clinic, population_ecology, evolutionary_algorithm, developmental_learner, environments, simulator)
