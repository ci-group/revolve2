from abc import ABC, abstractmethod

from revolve2.abstract.creational.builder import Builder
from revolve2.revolve.robot.body.body import Body, RobotBody
from revolve2.revolve.robot.development_request import BodyDevelopmentRequest


class BodyBuilder(Builder, ABC):
    def __init__(self, body_type: type(Body)):
        super().__init__()
        self.body_type: type(Body) = body_type

    @abstractmethod
    def create(self, body_development_request: BodyDevelopmentRequest):
        pass


class RobotBodyBuilder(BodyBuilder, ABC):

    def __init__(self, body_type: type(Body) = RobotBody):
        super().__init__(body_type)

    def create(self, body_development_request: BodyDevelopmentRequest):
        return self.body_type(body_development_request)
