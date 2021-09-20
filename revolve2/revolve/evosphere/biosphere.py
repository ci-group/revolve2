from abc import ABC
from typing import List

from revolve2.nca.core.actor.individual_factory import ActorFactory
from revolve2.nca.core.ecology import PopulationEcology
from revolve2.nca.core.ecology.population import Population
from revolve2.nca.core.evolution.conditions.initialization import Initialization

from revolve2.revolve.evosphere.ecosphere import GazeboEcosphere, Ecosphere
from revolve2.revolve.robot.birth_clinic import RobotBirthClinic, BirthClinic, IndividualBirthClinic


class Biosphere(ABC):

    def __init__(self,
                 population_ecology: PopulationEcology = PopulationEcology(),
                 actor_factory: ActorFactory = ActorFactory(),
                 birth_clinic: BirthClinic = IndividualBirthClinic(),
                 ecospheres: List[Ecosphere] = None):
        self.population_ecology: PopulationEcology = population_ecology
        self.actor_factory: ActorFactory = actor_factory
        self.birth_clinic: BirthClinic = birth_clinic
        self.ecospheres: List[Ecosphere] = ecospheres if ecospheres is not None else [GazeboEcosphere()]

    def initialize(self, number_of_actors: int, initialization_type: type(Initialization)):
        if False:  # TODO
            self.population_ecology.load()
        else:
            self.population_ecology.initialize(self.actor_factory.create(number_of_actors))

    def populations(self) -> List[Population]:
        return self.population_ecology.management.populations()

    def run(self):
        self.population_ecology.export()
        self.population_ecology.process()


class RobotBiosphere(Biosphere):

    def __init__(self,
                 population_ecology: PopulationEcology = PopulationEcology(),
                 actor_factory: ActorFactory = ActorFactory(),
                 birth_clinic: BirthClinic = RobotBirthClinic(),
                 ecospheres: List[Ecosphere] = None):
        super().__init__(population_ecology, actor_factory, birth_clinic, ecospheres)


class IndividualBiosphere(Biosphere):

    def __init__(self,
                 population_ecology: PopulationEcology = PopulationEcology(),
                 actor_factory: ActorFactory = ActorFactory(),
                 birth_clinic: BirthClinic = IndividualBirthClinic(),
                 ecospheres: List[Ecosphere] = None):
        super().__init__(population_ecology, actor_factory, birth_clinic, ecospheres)
