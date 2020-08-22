from abc import ABC
from typing import List

from nca.core.agent.individual_factory import ActorFactory, IndividualFactory
from nca.core.ecology import PopulationEcology
from nca.core.ecology.population import Population
from nca.core.evolution.conditions.initialization import Initialization
from nca.core.genome.initialization import IntegerInitialization
from nca.core.genome.representations.valued_representation import ValuedRepresentation
from revolve.evosphere.ecosphere import Ecosphere, GazeboEcosphere, GeneticOnesEcosphere
from revolve.robot.birth_clinic import RobotFactory
from revolve.robot.brain.representation.multineat_representation import MultiNEATRepresentation
from revolve.robot.robogen.robogen_representation import RobogenRepresentation


class Biosphere(ABC):

    def __init__(self, population_ecology: PopulationEcology, actor_factory: ActorFactory,
                 ecospheres: List[Ecosphere] = None):
        self.population_ecology: PopulationEcology = population_ecology
        self.actor_factory: ActorFactory = actor_factory

        self.ecospheres: List[Ecosphere] = ecospheres
        if self.ecospheres is None:
            self.ecospheres = [GazeboEcosphere()]

    def initialize(self, number_of_agents, initialization: Initialization):
        if False:  # TODO
            self.population_ecology.load()
        else:
            self.actor_factory.initialize(initialization = initialization)
            self.population_ecology.initialize(self.actor_factory.create(number_of_agents))

    def populations(self) -> List[Population]:
        return self.population_ecology.management.populations()

    def run(self):
        self.population_ecology.export()
        self.population_ecology.speciate()


class RevolveRobotBiosphere(Biosphere):

    def __init__(self):
        super().__init__(PopulationEcology(), RobotFactory(RobogenRepresentation, MultiNEATRepresentation))


class GeneticBiosphere(Biosphere):

    def __init__(self, ecospheres: List[Ecosphere] = None):
        super().__init__(PopulationEcology(), IndividualFactory(ValuedRepresentation),
                         ecospheres if ecospheres is not None else [GeneticOnesEcosphere()])

