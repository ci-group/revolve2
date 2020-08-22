from abc import ABC, abstractmethod

from nca.core.genome.representation import Representation
from revolve.robot.morphology import Morphology


class BrainRepresentation(Representation, ABC):

    @abstractmethod
    def develop(self):
        pass


class Brain(Morphology, Representation):

    def __init__(self, representation: BrainRepresentation):
        super().__init__(representation)
        self.phenotype = None

