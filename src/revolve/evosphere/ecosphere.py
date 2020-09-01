import os
import string

from nca.core.actor.fitness import Fitness
from nca.core.actor.fitnesses import DisplacementFitness
from nca.core.actor.individual import Individual
from nca.experiment_manager import ExperimentManager
from revolve.evosphere.performance_measures import PerformanceMeasures
from simulation.simulator.simulator_helper import SimulatorType


class Ecosphere:

    experiment_manager = ExperimentManager()

    def __init__(self, filename: string, fitness_type: type(Fitness) = DisplacementFitness,
                 simulator_type: SimulatorType = SimulatorType.NONE):
        self.path: string = os.path.join(self.experiment_manager.world_path, simulator_type.name + "/" + filename)
        self.fitness: Fitness = fitness_type()
        self.simulator_type: SimulatorType = simulator_type
        self.birth_clinic = None

    def initialize(self, birth_clinic):
        self.birth_clinic = birth_clinic


class GazeboEcosphere(Ecosphere):
    def __init__(self, filename: string = "plane", fitness_type: type(Fitness) = DisplacementFitness):
        super().__init__(filename, fitness_type, SimulatorType.GAZEBO)

    def run(self, individual: Individual):
        individual.measures = PerformanceMeasures()
        return individual.performance(self.fitness)


class UnityEcosphere(Ecosphere):
    def __init__(self, filename: string, fitness_type: type(Fitness) = DisplacementFitness):
        super().__init__(filename, fitness_type, SimulatorType.Unity)


class VREPEcosphere(Ecosphere):
    def __init__(self, filename: string, fitness_type: type(Fitness) = DisplacementFitness):
        super().__init__(filename, fitness_type, SimulatorType.V_REP)


class MalmoEcosphere(Ecosphere):
    def __init__(self, filename: string, fitness_type: type(Fitness) = DisplacementFitness):
        super().__init__(filename, fitness_type, SimulatorType.MALMO)
