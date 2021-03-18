from abc import abstractmethod, ABC
from typing import List

from abstract.creational.factory import Factory
from nca.core.actor.agent import Agent
from nca.core.actor.individual import Individual
from nca.core.actor.actors import Actors
from revolve.robot.body.body import RobotBody

from revolve.robot.body.body_builder import BodyBuilder, RobotBodyBuilder
from revolve.robot.brain.brain import RobotBrain
from revolve.robot.brain.brain_builder import BrainBuilder, RobotBrainBuilder, AgentBrainBuilder
from revolve.robot.development_request import DevelopmentRequest, BrainDevelopmentRequest, BodyDevelopmentRequest
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

    def __init__(self, body_builder: RobotBodyBuilder, brain_builder: RobotBrainBuilder):
        super().__init__()
        self.body_builder: RobotBodyBuilder = body_builder
        self.brain_builder: RobotBrainBuilder = brain_builder

    def create(self, agents: Actors, ecosphere):
        robots: List[Robot] = []
        for agent in agents:
            robots.append(self.build(agent, ecosphere))
        return robots

    def build(self, agent: Individual, ecosphere) -> Robot:
        body_development_request = BodyDevelopmentRequest(agent.id,
                                                          agent.genotype.get('body'),
                                                          ecosphere)
        body: RobotBody = self.body_builder.create(body_development_request)

        brain_development_request = BrainDevelopmentRequest(agent.id,
                                                            agent.genotype.get('brain'),
                                                            ecosphere,
                                                            body.get_blueprint())
        brain: RobotBrain = self.brain_builder.create(brain_development_request)

        return Robot(agent.genotype, body=body, brain=brain)
