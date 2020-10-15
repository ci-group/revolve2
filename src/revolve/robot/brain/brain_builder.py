from abc import ABC

from evosphere.robot.mock_morphology import MockBrain
from nca.core.actor.individual import Individual
from revolve.evosphere.ecosphere import Ecosphere
from revolve.robot.brain.brain import Brain, RobotBrain, AgentBrain
from revolve.robot.robot_builder import RobotBuilder


class BrainBuilder(RobotBuilder, ABC):
    def __init__(self, brain_type: type(Brain)):
        super().__init__()
        self.brain_type: type(Brain) = brain_type


class RobotBrainBuilder(BrainBuilder):
    def __init__(self, brain_type: type(Brain) = RobotBrain):
        super().__init__(brain_type)

    def build(self, individual: Individual, ecosphere: Ecosphere) -> Brain:
        return self.brain_type()


class AgentBrainBuilder(BrainBuilder):
    def __init__(self, brain_type: type(Brain) = AgentBrain):
        super().__init__(brain_type)

    def build(self, individual: Individual, ecosphere: Ecosphere) -> Brain:
        return self.brain_type()


class MockBrainBuilder(BrainBuilder):
    def __init__(self, brain_type: type(Brain) = MockBrain):
        super().__init__(brain_type)

    def build(self, individual: Individual, ecosphere: Ecosphere) -> Brain:
        return self.brain_type()
