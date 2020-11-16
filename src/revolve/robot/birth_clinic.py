from abc import abstractmethod

from nca.core.abstract.creational.builder import Builder
from nca.core.actor.individual import Individual
from nca.core.actor.actors import Actors
from revolve.evosphere.ecosphere import Ecosphere
from revolve.robot.body.body_builder import BodyBuilder, RobotBodyBuilder
from revolve.robot.brain.brain_builder import BrainBuilder, RobotBrainBuilder

from revolve.robot.robot import Robot


class BirthClinic(Builder):

    def __init__(self):
        super().__init__()

    def build(self, agents: Actors, ecosphere: Ecosphere) -> Actors:
        robots: Actors = Actors()

        for individual in agents:
            robots.add(self._create(individual, ecosphere))

        return robots

    @abstractmethod
    def _create(self, individual: Individual, ecosphere: Ecosphere) -> Individual:
        pass


class IndividualBirthClinic(BirthClinic):

    def __init__(self):
        super().__init__()

    def _create(self, individual: Individual, ecosphere: Ecosphere) -> Individual:
        return individual


class RobotBirthClinic(BirthClinic):

    def __init__(self, brain_builder: BrainBuilder = RobotBrainBuilder(),
                 body_builder: BodyBuilder = RobotBodyBuilder()):
        super().__init__()
        self.brain_builder: BrainBuilder = brain_builder
        self.body_builder: BodyBuilder = body_builder

    def _create(self, individual: Individual, ecosphere: Ecosphere) -> object:
        robot = Robot(individual.genotype)

        individual.genotype()

        if self.brain_builder is not None:
            robot.brain = self.brain_builder.build(individual.genotype, ecosphere)

        if self.body_builder is not None:
            robot.body = self.body_builder.build(individual.genotype, ecosphere)

        return robot
