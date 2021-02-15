from abc import abstractmethod, ABC

from abstract.creational.factory import Factory
from nca.core.actor.agent import Agent
from nca.core.actor.individual import Individual
from nca.core.actor.actors import Actors

from revolve.robot.body.body_builder import BodyBuilder, RobotBodyBuilder
from revolve.robot.brain.brain_builder import BrainBuilder, RobotBrainBuilder, AgentBrainBuilder
from revolve.robot.development_request import DevelopmentRequest, BrainDevelopmentRequest
from revolve.robot.robot import Robot


class BirthClinic(Factory, ABC):

    def __init__(self):
        super().__init__()

    def create(self, agents: Actors, ecosphere) -> Actors:
        robots: Actors = Actors()

        for individual in agents:
            robots.append(self._create(DevelopmentRequest(individual.id, individual.genotype, ecosphere)))

        return robots

    @abstractmethod
    def _create(self, development_request: DevelopmentRequest) -> Individual:
        pass


class IndividualBirthClinic(BirthClinic):

    def __init__(self):
        super().__init__()

    def _create(self, development_request: DevelopmentRequest) -> Individual:
        return Individual(development_request.genotype)


class AgentBirthClinic(BirthClinic):
    def __init__(self, brain_builder: BrainBuilder = AgentBrainBuilder()):
        super().__init__()
        self.brain_builder: BrainBuilder = brain_builder

    def _create(self, development_request: BrainDevelopmentRequest) -> object:
        return Agent(development_request.genotype, self.brain_builder.create(development_request))


class RobotBirthClinic(BirthClinic):

    def __init__(self, brain_builder: BrainBuilder = RobotBrainBuilder(),
                 body_builder: BodyBuilder = RobotBodyBuilder()):
        super().__init__()
        self.brain_builder: BrainBuilder = brain_builder
        self.body_builder: BodyBuilder = body_builder

    def _create(self, development_request: DevelopmentRequest) -> object:
        robot = Robot(development_request.genotype)

        if self.brain_builder is not None:
            robot.brain = self.brain_builder.create(development_request)

        if self.body_builder is not None:
            robot.body = self.body_builder.create(development_request)

        return robot
