from abc import ABC
from typing import List

from nca.core.actor.individual_factory import IndividualFactory
from nca.core.ecology import PopulationEcology
from nca.core.ecology.population import Population
from nca.core.evolution.conditions.initialization import Initialization
from revolve.evosphere.ecosphere import GazeboEcosphere, Ecosphere
from revolve.robot.birth_clinic import AgentBirthClinic, RobotBirthClinic, BirthClinic
from simulation.simulator.simulator_command import SimulateCommand


class Biosphere(ABC):

    def __init__(self,
                 population_ecology: PopulationEcology = PopulationEcology(),
                 actor_factory: IndividualFactory = IndividualFactory(),
                 birth_clinic: BirthClinic = BirthClinic(),
                 ecospheres: List[Ecosphere] = None):
        self.population_ecology: PopulationEcology = population_ecology
        self.actor_factory: IndividualFactory = actor_factory
        self.birth_clinic: BirthClinic = birth_clinic
        self.ecospheres: List[Ecosphere] = ecospheres if ecospheres is not None else [GazeboEcosphere()]

    def initialize(self, number_of_actors: int, initialization_type: type(Initialization)):
        if False:  # TODO
            self.population_ecology.load()
        else:
            self.actor_factory.initialize(initialization_type)
            self.population_ecology.initialize(self.actor_factory.create(number_of_actors))

    def populations(self) -> List[Population]:
        return self.population_ecology.management.populations()

    def run(self):
        self.population_ecology.export()
        self.population_ecology.speciate()


class RobotBiosphere(Biosphere):

    def __init__(self,
                 population_ecology: PopulationEcology = PopulationEcology(),
                 actor_factory: IndividualFactory = IndividualFactory(),
                 birth_clinic: BirthClinic = RobotBirthClinic(),
                 ecospheres: List[Ecosphere] = None):
        super().__init__(population_ecology, actor_factory, birth_clinic, ecospheres)


class AgentBiosphere(Biosphere):

    def __init__(self,
                 population_ecology: PopulationEcology = PopulationEcology(),
                 actor_factory: IndividualFactory = IndividualFactory(),
                 birth_clinic: BirthClinic = AgentBirthClinic(),
                 ecospheres: List[Ecosphere] = None):
        super().__init__(population_ecology, actor_factory, birth_clinic, ecospheres)
