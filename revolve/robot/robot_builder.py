from abc import abstractmethod, ABC

from nca.core.genome.representation import Representation
from revolve.robot.morphology import Morphology


class RobotBuilder(ABC):

    def __init__(self, representation: type(Representation)):
        self.representation: type(Representation) = representation

    @abstractmethod
    def build(self) -> Morphology:
        pass
