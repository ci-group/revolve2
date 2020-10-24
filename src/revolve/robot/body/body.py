from abc import abstractmethod, ABC

from nca.core.genome.representation import MorphologyRepresentation


class Body(ABC):
    @abstractmethod
    def develop(self, representation: MorphologyRepresentation, ecosphere):
        pass


class RobotBody(Body):
    def develop(self, representation: MorphologyRepresentation, ecosphere):
        pass

