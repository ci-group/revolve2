from abc import abstractmethod, ABC

from nca.core.genome.representation import MorphologyRepresentation


class Brain(ABC):
    @abstractmethod
    def develop(self, representation: MorphologyRepresentation, ecosphere):
        pass


class RobotBrain(Brain):
    def develop(self, representation: MorphologyRepresentation, ecosphere):
        pass


class AgentBrain(Brain):
    def develop(self, representation: MorphologyRepresentation, ecosphere):
        pass
