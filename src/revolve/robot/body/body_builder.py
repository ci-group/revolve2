from abc import ABC, abstractmethod

from evosphere.robot.mock_morphology import MockBody
from nca.core.genome.genotype import Genotype
from revolve.evosphere.ecosphere import Ecosphere
from revolve.robot.body.body import Body, RobotBody
from revolve.robot.robot_builder import RobotBuilder


class BodyBuilder(RobotBuilder, ABC):
    def __init__(self, body_type: type(Body)):
        super().__init__()
        self.body_type: type(Body) = body_type

    @abstractmethod
    def build(self, genotype: Genotype, ecosphere: Ecosphere):
        pass


class RobotBodyBuilder(BodyBuilder, ABC):

    def __init__(self, body_type: type(Body) = RobotBody):
        super().__init__(body_type)

    def build(self, genotype: Genotype, ecosphere: Ecosphere):
        return self.body_type()


class MockBodyBuilder(BodyBuilder, ABC):

    def __init__(self, body_type: type(Body) = MockBody):
        super().__init__(body_type)

    def build(self, genotype: Genotype, ecosphere: Ecosphere):
        return self.body_type()