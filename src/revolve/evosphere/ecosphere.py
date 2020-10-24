import os
import string

from nca.core.actor.fitness import Fitness
from nca.core.actor.fitnesses import DisplacementFitness, FitnessEvaluation
from nca.core.actor.individual import Individual
from nca.experiment_manager import ExperimentManager
from revolve.evosphere.performance_measures import PerformanceMeasures
from simulation.simulator.simulator_helper import SimulatorType


class Ecosphere:

    experiment_manager = ExperimentManager()

    def __init__(self, filename: string, fitness_evaluation_type: type(FitnessEvaluation) = DisplacementFitness,
                 simulator_type: SimulatorType = SimulatorType.NONE):
        self.path: string = os.path.join(self.experiment_manager.world_path, simulator_type.name + "/" + filename)
        if fitness_evaluation_type is not None:
            self.fitness_evaluation: FitnessEvaluation = fitness_evaluation_type()
        self.simulator_type: SimulatorType = simulator_type


class GazeboEcosphere(Ecosphere):
    def __init__(self, filename: string = "plane", fitness_evaluation_type: type(FitnessEvaluation) = DisplacementFitness):
        super().__init__(filename, fitness_evaluation_type, SimulatorType.GAZEBO)

    def run(self, individual: Individual):
        individual.measures = PerformanceMeasures()
        return self.fitness_evaluation(individual)


class UnityEcosphere(Ecosphere):
    def __init__(self, filename: string, fitness_evaluation_type: type(FitnessEvaluation) = DisplacementFitness):
        super().__init__(filename, fitness_evaluation_type, SimulatorType.Unity)


class VREPEcosphere(Ecosphere):
    def __init__(self, filename: string, fitness_evaluation_type: type(FitnessEvaluation) = DisplacementFitness):
        super().__init__(filename, fitness_evaluation_type, SimulatorType.V_REP)


class MalmoEcosphere(Ecosphere):
    def __init__(self, filename: string, fitness_evaluation_type: type(FitnessEvaluation) = DisplacementFitness):
        super().__init__(filename, fitness_evaluation_type, SimulatorType.MALMO)
