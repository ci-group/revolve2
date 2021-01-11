from abc import ABC

from test.evosphere.robot.mock_morphology import MockBrain
from revolve.robot.brain.brain import Brain, RobotBrain, AgentBrain
from revolve.robot.development_request import BrainDevelopmentRequest
from revolve.robot.robot_builder import RobotBuilder


class BrainBuilder(RobotBuilder, ABC):
    def __init__(self, brain_type: type(Brain)):
        super().__init__()
        self.brain_type: type(Brain) = brain_type


class RobotBrainBuilder(BrainBuilder):
    def __init__(self, brain_type: type(Brain) = RobotBrain):
        super().__init__(brain_type)

    def create(self, brain_development_request: BrainDevelopmentRequest) -> Brain:
        return self.brain_type(brain_development_request)


class AgentBrainBuilder(BrainBuilder):
    def __init__(self, brain_type: type(Brain) = AgentBrain):
        super().__init__(brain_type)

    def create(self, brain_development_request: BrainDevelopmentRequest) -> Brain:
        return self.brain_type(brain_development_request)


class MockBrainBuilder(BrainBuilder):
    def __init__(self, brain_type: type(Brain) = MockBrain):
        super().__init__(brain_type)

    def create(self, brain_development_request: BrainDevelopmentRequest) -> Brain:
        return self.brain_type()
