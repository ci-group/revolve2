from abc import abstractmethod

from nca.core.abstract.creational.builder import Builder
from nca.core.actor.agent import Agent
from nca.core.actor.individual import Individual
from nca.core.actor.actors import Actors
from revolve.evosphere.ecosphere import Ecosphere
from revolve.robot.body.body_builder import BodyBuilder, RobotBodyBuilder
from revolve.robot.brain.brain_builder import BrainBuilder, RobotBrainBuilder, AgentBrainBuilder

from revolve.robot.robot import Robot


class BirthClinic(Builder):

    def __init__(self):
        super().__init__()

    def build(self, agents: Actors, ecosphere: Ecosphere) -> Actors:
        robots: Actors = Actors()

        for agent in agents:
            robots.add(self.develop(agent, ecosphere))

        return robots

    @abstractmethod
    def develop(self, individual: Individual, ecosphere: Ecosphere) -> Individual:
        pass


class IndividualBirthClinic(BirthClinic):

    def __init__(self):
        super().__init__()

    def develop(self, individual: Individual, ecosphere: Ecosphere) -> Individual:
        return individual


class RobotBirthClinic(BirthClinic):

    def __init__(self):
        super().__init__()
        self.brain_builder: BrainBuilder = RobotBrainBuilder()
        self.body_builder: BodyBuilder = RobotBodyBuilder()

    def develop(self, individual: Individual, ecosphere: Ecosphere) -> object:
        robot = Robot(individual.genotype)
        robot.brain = self.brain_builder.build(individual, ecosphere)
        robot.body = self.body_builder.build(individual, ecosphere)
        return robot


class AgentBirthClinic(BirthClinic):

    def __init__(self, brain_builder: BrainBuilder = AgentBrainBuilder):
        super().__init__()
        self.brain_builder: BrainBuilder = brain_builder

    def develop(self, individual: Individual, ecosphere: Ecosphere) -> object:
        agent = Agent(individual)
        agent.brain = self.brain_builder.build(individual=individual, ecosphere=ecosphere)
        return agent
