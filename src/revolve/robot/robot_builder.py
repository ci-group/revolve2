from abc import abstractmethod, ABC

from nca.core.abstract.creational.builder import Builder
from nca.core.genome.representation import Representation
from revolve.robot.morphology import Morphology


class RobotBuilder(Builder, ABC):

    def __init__(self, representation: type(Representation)):
        super().__init__()
        self.representation: type(Representation) = representation

    @abstractmethod
    def build(self) -> Morphology:
        pass
