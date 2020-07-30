from abc import abstractmethod, ABC

from pyrevolve.evolutionary.algorithm.genome.representation import Representation
from pyrevolve.evolutionary.robotics.morphology.morphology import Morphology


class RobotBuilder(ABC):

    def __init__(self, representation: type(Representation)):
        self.representation: type(Representation) = representation

    @abstractmethod
    def build(self) -> Morphology:
        pass
