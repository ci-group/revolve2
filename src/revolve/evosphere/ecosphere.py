import os
import string
from abc import ABC, abstractmethod

from nca.core.agent.fitness import DisplacementFitness, Fitness, OnesFitness
from nca.core.agent.individual import Individual
from nca.experiment_manager import ExperimentManager
from simulation.simulator.simulator_helper import SimulatorType


class Ecosphere:

    experiment_manager = ExperimentManager()

    def __init__(self, filename: string, fitness_type: type(Fitness) = DisplacementFitness,
                 simulator_type: SimulatorType = SimulatorType.NONE):
        self.path: string = os.path.join(self.experiment_manager.world_path, simulator_type.name + "/" + filename)
        self.fitness = fitness_type()
        self.simulator_type: SimulatorType = simulator_type


class GeneticEcosphere(Ecosphere, ABC):
    def __init__(self, filename: string, fitness_type: type(Fitness) = DisplacementFitness):
        super().__init__(filename, fitness_type, SimulatorType.GENETIC)

    @abstractmethod
    def run(self, individual: Individual):
        pass


class GeneticOnesEcosphere(GeneticEcosphere):
    def __init__(self, filename: string = "ones", fitness_type: type(Fitness) = OnesFitness):
        super().__init__(filename, fitness_type)

    def run(self, individual: Individual):
        return self.fitness(individual)


class SimulationEcosphere(Ecosphere):
    pass


class GazeboEcosphere(SimulationEcosphere):
    def __init__(self, filename: string = "plane", fitness_type: type(Fitness) = DisplacementFitness):
        super().__init__(filename, fitness_type, SimulatorType.GAZEBO)


class UnityEcosphere(SimulationEcosphere):
    def __init__(self, filename: string, fitness_type: type(Fitness) = DisplacementFitness):
        super().__init__(filename, fitness_type, SimulatorType.Unity)


class VREPEcosphere(SimulationEcosphere):
    def __init__(self, filename: string, fitness_type: type(Fitness) = DisplacementFitness):
        super().__init__(filename, fitness_type, SimulatorType.V_REP)


class MalmoEcosphere(SimulationEcosphere):
    def __init__(self, filename: string, fitness_type: type(Fitness) = DisplacementFitness):
        super().__init__(filename, fitness_type, SimulatorType.MALMO)
