from nca.core.agent.agents import Agents
from nca.core.agent.fitness import Fitness
from nca.core.genome.grammar.lsystem_representation import LSystemRepresentation
from nca.core.genome.representation import Representation
from revolve.robot.body.body_builder import BodyBuilder
from src.revolve.robot.brain.brain_builder import BrainBuilder
from revolve.robot.brain.representation.multineat_representation import MultiNEATRepresentation

from revolve.robot.robot import Robot


class BirthClinic:

    def __init__(self, body_genome: BodyBuilder = BodyBuilder(Representation),
                 brain_genome: BrainBuilder = BrainBuilder(Representation),
                 fitness: type(Fitness) = Fitness):

        self.body_builder: BodyBuilder = body_genome
        self.brain_builder: BrainBuilder = brain_genome
        self.fitness_type: type(Fitness) = fitness

    def create(self, number_of_robots: int) -> Agents:
        robots: Agents = Agents()

        for robot_index in range(number_of_robots):
            robots.add(self.build_robot())

        return robots

    def build_robot(self) -> Robot:
        return Robot(self.body_builder.build(), self.brain_builder.build(), self.fitness_type())


class DefaultLSystemCPGMultiNEATClinic(BirthClinic):

    def __init__(self):
        super().__init__(BodyBuilder(LSystemRepresentation), BrainBuilder(MultiNEATRepresentation))
