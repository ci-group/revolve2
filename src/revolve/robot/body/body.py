from abc import ABC, abstractmethod

from nca.core.genome.representation import Representation
from revolve.robot.morphology import Morphology


class BodyRepresentation(Representation, ABC):
    @abstractmethod
    def develop(self):
        pass


class Body(Morphology):

    def __init__(self, genome: BodyRepresentation):
        super().__init__(genome)
        self.phenotype = None

