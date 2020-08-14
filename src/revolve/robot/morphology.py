from abc import ABC
from enum import Enum, auto

from nca.core.genome.representation import Representation


class MorphologyType(Enum):
    BODY = auto()
    BRAIN = auto()


class Morphology(ABC):

    def __init__(self, genome: Representation):
        self.genome = genome
