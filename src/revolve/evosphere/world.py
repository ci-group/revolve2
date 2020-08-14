import os
import string
from abc import ABC
from typing import List

from nca.core.agent.fitness import DisplacementFitness, Fitness
from nca.core.agent.individual_factory import ActorFactory
from nca.core.ecology import PopulationEcology
from nca.core.ecology.population import Population
from nca.experiment_manager import ExperimentManager
from simulation.simulator.simulator_helper import SimulatorType


class Ecosphere:

    experiment_manager = ExperimentManager()

    def __init__(self, filename: string, fitness_type: type(Fitness) = DisplacementFitness,
                 simulator_type: SimulatorType = SimulatorType.NONE):
        self.path: string = os.path.join(self.experiment_manager.world_path, filename)
        self.fitness_type = fitness_type
        self.simulator_type: SimulatorType = simulator_type


class Biosphere(ABC):

    def __init__(self, population_ecology: PopulationEcology, actor_factory: ActorFactory,
                 ecosphere: List[Ecosphere] = None, simulator_type: SimulatorType = SimulatorType.NONE):
        self.population_ecology: PopulationEcology = population_ecology
        self.actor_factory: ActorFactory = actor_factory
        self.simulator_type: SimulatorType = simulator_type

        self.ecosphere: List[Ecosphere] = ecosphere
        if self.ecosphere is None:
            self.ecosphere = [Ecosphere(os.path.join(self.simulator_type.name, "plane"), DisplacementFitness)]

    def initialize(self, number_of_agents):
        if False:  # TODO
            self.population_ecology.load()
        else:
            self.population_ecology.initialize(self.actor_factory.create(number_of_agents))

    def populations(self) -> List[Population]:
        return self.population_ecology.management.populations()

    def run(self):
        self.population_ecology.export()
        self.population_ecology.speciate()

"""
class GazeboEcosphere(Ecosphere):
    def __init__(self, ):
        super().__init__(population_ecology, birth_clinic, environments, SimulatorType.GAZEBO)


class UnityEcosphere(Ecosphere):
    def __init__(self, population_ecology: PopulationEcology = PopulationEcology(),
                 birth_clinic: RobotFactory = RobotFactory(BodyBuilder(RobogenRepresentation),
                                                           BrainBuilder(MultiNEATRepresentation)),
                 environments: List[Environment] = None):
        super().__init__(population_ecology, birth_clinic, environments, SimulatorType.Unity)


class VREPEcosphere(Ecosphere):
    def __init__(self, population_ecology: PopulationEcology = PopulationEcology(),
                 birth_clinic: RobotFactory = RobotFactory(BodyBuilder(RobogenRepresentation),
                                                           BrainBuilder(MultiNEATRepresentation)),
                 environments: List[Environment] = None):
        super().__init__(population_ecology, birth_clinic, environments, SimulatorType.V_REP)


class MalmoBiosphere(Biosphere):
    def __init__(self, population_ecology: PopulationEcology = PopulationEcology(),
                 birth_clinic: RobotFactory = RobotFactory(None, BrainBuilder(MultiNEATRepresentation)),
                 environments: List[Environment] = None):
        super().__init__(population_ecology, birth_clinic, environments, SimulatorType.MALMO)
"""
