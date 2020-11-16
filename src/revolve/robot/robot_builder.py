from abc import abstractmethod, ABC

from nca.core.abstract.creational.builder import Builder
from nca.core.actor.individual import Individual
from nca.core.genome.genotype import Genotype
from revolve.evosphere.ecosphere import Ecosphere


class RobotBuilder(Builder, ABC):

    def __init__(self):
        super().__init__()

    @abstractmethod
    def build(self, genotype: Genotype, ecosphere: Ecosphere):
        pass

