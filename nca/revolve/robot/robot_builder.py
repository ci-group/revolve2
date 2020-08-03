from abc import abstractmethod, ABC

from nca.core.genome import Representation
from nca.revolve.robot.morphology.morphology import Morphology


class RobotBuilder(ABC):

    def __init__(self, representation: type(Representation)):
        self.representation: type(Representation) = representation

    @abstractmethod
    def build(self) -> Morphology:
        pass
