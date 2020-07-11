from typing import List

from pyrevolve.developmental.developmental_learning import DevelopmentalLearner
from pyrevolve.evolutionary.algorithm.ecology.population import Population
from pyrevolve.evolutionary.algorithm.evolutionary_algorithm import EvolutionaryAlgorithm
from pyrevolve.evolutionary.things.birth_clinic import BirthClinic
from pyrevolve.evolutionary.things.environment import Environment
from pyrevolve.shared.configurations import EvoSphereConfiguration

from pyrevolve.simulator.simulator import Simulator


class EvoSphere:

    def __init__(self,
                 birth_clinic: BirthClinic,
                 evolutionary_algorithm: EvolutionaryAlgorithm,
                 developmental_learner: DevelopmentalLearner,
                 environments: List[Environment],
                 simulator: Simulator):
        self.configuration = EvoSphereConfiguration()

        self.birth_clinic: BirthClinic = birth_clinic

        self.evolutionary_algorithm: EvolutionaryAlgorithm = evolutionary_algorithm
        self.developmental_leaner: DevelopmentalLearner = developmental_learner

        self.environments: List[Environment] = environments

        self.simulator = simulator

    def evolve(self, populations: List[Population]):
        pass


class DefaultEvoSphere(EvoSphere):

    def __init__(self,
                 birth_clinic: BirthClinic = BirthClinic(),
                 evolutionary_algorithm: EvolutionaryAlgorithm = EvolutionaryAlgorithm(),
                 developmental_learner: DevelopmentalLearner = DevelopmentalLearner(),
                 environments: List[Environment] = [],
                 simulator: Simulator = Simulator()):
        super().__init__(birth_clinic, evolutionary_algorithm, developmental_learner, environments, simulator)
