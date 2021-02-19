from abc import abstractmethod, ABC

from abstract.creational.builder import Builder
from revolve.robot.body.body import RobotBody
from revolve.robot.body.body_blueprint import BodyBlueprint
from revolve.robot.body.body_builder import RobotBodyBuilder
from revolve.robot.brain.brain import Brain, RobotBrain
from revolve.robot.brain.brain_builder import RobotBrainBuilder
from revolve.robot.development_request import DevelopmentRequest, BodyDevelopmentRequest, BrainDevelopmentRequest
from revolve.robot.robot import Robot


class RobotBuilder(Builder, ABC):

    def __init__(self, body_builder: RobotBodyBuilder, brain_builder: RobotBrainBuilder):
        super().__init__()
        self.body_builder: RobotBodyBuilder = body_builder
        self.brain_builder: RobotBrainBuilder = brain_builder

    def create(self, development_request: DevelopmentRequest):
        body_development_request = BodyDevelopmentRequest(development_request.individual_identifier,
                                                          development_request.genotype, development_request.ecosphere)
        body: RobotBody = self.body_builder.create(body_development_request)

        brain_development_request = BrainDevelopmentRequest(development_request.individual_identifier,
                                                            development_request.genotype, development_request.ecosphere,
                                                            body.get_blueprint())
        brain: RobotBrain = self.brain_builder.create(brain_development_request)

        robot = Robot(development_request.genotype, body=body, brain=brain)
        return robot
