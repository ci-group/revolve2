from abc import ABC, abstractmethod

from nca.core.genome.representation import Representation
from revolve.robot.morphology import Morphology


class BrainRepresentation(Representation, ABC):

    @abstractmethod
    def develop(self):
        pass


class Brain(Morphology):

    def __init__(self, genome: BrainRepresentation):
        super().__init__(genome)
        self.phenotype = None

