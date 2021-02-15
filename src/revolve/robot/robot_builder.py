from abc import abstractmethod, ABC

from abstract.creational.builder import Builder
from revolve.robot.brain.brain import Brain
from revolve.robot.development_request import DevelopmentRequest, BrainDevelopmentRequest


class RobotBuilder(Builder, ABC):

    def __init__(self):
        super().__init__()

    @abstractmethod
    def create(self, brain_development_request: BrainDevelopmentRequest) -> Brain:
        pass
