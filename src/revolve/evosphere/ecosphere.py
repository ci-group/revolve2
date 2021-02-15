import os
import string

from nca.core.actor.fitness_evaluation import FitnessEvaluation
from nca.core.actor.individual import Individual
from nca.experiment_manager import ExperimentManager
from revolve.evosphere.fitness_evaluation import DisplacementFitnessEvaluation
from revolve.evosphere.performance_measures import PerformanceMeasures
from simulation.simulator.simulator_type import SimulatorType


class Ecosphere:

    experiment_manager = ExperimentManager()

    def __init__(self, filename: string, fitness_evaluation_type: type(FitnessEvaluation) = DisplacementFitnessEvaluation,
                 simulator_type: SimulatorType = SimulatorType.NONE):
        super().__init__()
        self.path: string = os.path.join(self.experiment_manager.folders.world_path, simulator_type.name + "/" + filename)
        if fitness_evaluation_type is not None:
            self.fitness_evaluation: FitnessEvaluation = fitness_evaluation_type()
        self.simulator_type: SimulatorType = simulator_type


class GazeboEcosphere(Ecosphere):
    def __init__(self, filename: string = "plane", fitness_evaluation_type: type(FitnessEvaluation) = DisplacementFitnessEvaluation):
        super().__init__(filename, fitness_evaluation_type, SimulatorType.GAZEBO)

    def run(self, individual: Individual):
        individual.measures = PerformanceMeasures()
        return self.fitness_evaluation(individual)


class UnityEcosphere(Ecosphere):
    def __init__(self, filename: string, fitness_evaluation_type: type(FitnessEvaluation) = DisplacementFitnessEvaluation):
        super().__init__(filename, fitness_evaluation_type, SimulatorType.Unity)


class CoppeliaEcosphere(Ecosphere):
    def __init__(self, filename: string, fitness_evaluation_type: type(FitnessEvaluation) = DisplacementFitnessEvaluation):
        super().__init__(filename, fitness_evaluation_type, SimulatorType.COPPELIA)


class MalmoEcosphere(Ecosphere):
    def __init__(self, filename: string, fitness_evaluation_type: type(FitnessEvaluation) = DisplacementFitnessEvaluation):
        super().__init__(filename, fitness_evaluation_type, SimulatorType.MALMO)
