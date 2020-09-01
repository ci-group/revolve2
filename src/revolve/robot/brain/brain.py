from abc import abstractmethod, ABC

from nca.core.genome.representation import MorphologyRepresentation
from revolve.evosphere.ecosphere import Ecosphere


class Brain(ABC):
    @abstractmethod
    def develop(self, representation: MorphologyRepresentation, ecosphere: Ecosphere):
        pass


class RobotBrain(Brain):
    pass


class AgentBrain(Brain):
    pass
