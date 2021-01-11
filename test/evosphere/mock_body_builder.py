from abc import ABC

from revolve.robot.body.body import Body
from revolve.robot.body.body_builder import BodyBuilder
from revolve.robot.development_request import DevelopmentRequest
from test.evosphere.robot.mock_morphology import MockBody


class MockBodyBuilder(BodyBuilder, ABC):

    def __init__(self, body_type: type(Body) = MockBody):
        super().__init__(body_type)

    def create(self, development_request: DevelopmentRequest):
        return self.body_type()
