from abc import ABC

from evosphere.robot.mock_morphology import MockBody
from nca.core.actor.individual import Individual
from revolve.evosphere.ecosphere import Ecosphere
from revolve.robot.body.body import Body, RobotBody
from revolve.robot.robot_builder import RobotBuilder


class BodyBuilder(RobotBuilder, ABC):
    def __init__(self, body_type: type(Body)):
        super().__init__()
        self.body_type: type(Body) = body_type


class RobotBodyBuilder(BodyBuilder):

    def __init__(self, body_type: type(Body) = RobotBody):
        super().__init__(body_type)

    def build(self, individual: Individual, ecosphere: Ecosphere):
        #genome = individual.express(ecosphere)
        return self.body_type()


class MockBodyBuilder(BodyBuilder):

    def __init__(self, body_type: type(Body) = MockBody):
        super().__init__(body_type)

    def build(self, individual: Individual, ecosphere: Ecosphere):
        return self.body_type()
