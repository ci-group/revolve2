from typing import List

from revolve2.nca.core.evolution.conditions.initialization import Initialization
from revolve2.nca.core.genome.representations.representation import Representation
from revolve2.revolve.robot.body.body_blueprint import BodyBlueprint


class NetworkInitialization(Initialization):

    def __init__(self, body_blueprint: BodyBlueprint, network_architecture: List):
        super().__init__()
        self.body_blueprint: BodyBlueprint = body_blueprint
        self.network_architecture: List = network_architecture


class NetworkRepresentation(Representation):

    def __init__(self, network_initialization: NetworkInitialization):
        super().__init__(network_initialization)


class MultiNEATRepresentation(NetworkRepresentation):

    def __init__(self, network_initialization: NetworkInitialization):
        super().__init__(network_initialization)

    def _initialize(self):
        pass

    def compatibility(self, other) -> float:
        pass
