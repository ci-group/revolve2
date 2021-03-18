from abc import ABC, abstractmethod

from abstract.creational.builder import Builder
from test.evosphere.robot.mock_morphology import MockBrain
from revolve.robot.brain.brain import Brain, RobotBrain, AgentBrain
from revolve.robot.development_request import BrainDevelopmentRequest



class BrainBuilder(Builder, ABC):
    def __init__(self, brain_type: type(Brain)):
        super().__init__()
        self.brain_type: type(Brain) = brain_type

    @abstractmethod
    def create(self, brain_development_request: BrainDevelopmentRequest) -> Brain:
        pass


class RobotBrainBuilder(BrainBuilder, ABC):
    def __init__(self, brain_type: type(Brain) = RobotBrain):
        super().__init__(brain_type)

    @abstractmethod
    def create(self, brain_development_request: BrainDevelopmentRequest) -> RobotBrain:
        pass


class AgentBrainBuilder(BrainBuilder, ABC):
    def __init__(self, brain_type: type(Brain) = AgentBrain):
        super().__init__(brain_type)

    @abstractmethod
    def create(self, brain_development_request: BrainDevelopmentRequest) -> AgentBrain:
        pass


class MockBrainBuilder(BrainBuilder):
    def __init__(self, brain_type: type(Brain) = MockBrain):
        super().__init__(brain_type)

    def create(self, brain_development_request: BrainDevelopmentRequest) -> MockBrain:
        return self.brain_type()
