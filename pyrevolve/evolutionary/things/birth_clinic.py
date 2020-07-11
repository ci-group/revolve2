from pyrevolve.evolutionary.agents import Agents
from pyrevolve.evolutionary.algorithm.genome.representation import Representation
from pyrevolve.evolutionary.algorithm.genome.representations.grammar_representation import GrammarRepresentation
from pyrevolve.evolutionary.algorithm.genome.representations.multineat_representation import MultiNEATRepresentation
from pyrevolve.evolutionary.robotics.morphology.body.body_builder import BodyBuilder
from pyrevolve.evolutionary.robotics.morphology.brain.brain_builder import BrainBuilder
from pyrevolve.evolutionary.robotics.robot import Robot
from pyrevolve.shared.configurations import RobotConfiguration
from pyrevolve.shared.sequential_identifier import SequentialIdentifier


class BirthClinic:

    def __init__(self, body_genome: Representation, brain_genome: Representation):
        self.robot_identifier = SequentialIdentifier()
        self.configuration = RobotConfiguration()

        self.body_builder = BodyBuilder(body_genome)
        self.brain_builder = BrainBuilder(brain_genome)

    def create_robots(self) -> Agents:
        robots: Agents = Agents()

        for robot_index in range(self.configuration.number_of_robots):
            robots.add(self.build_robot())

        return robots

    def build_robot(self) -> Robot:
        robot = Robot(self.robot_identifier.increment(), self.configuration.fitness)

        robot.body = self.body_builder.build()
        robot.brain = self.brain_builder.build()

        return robot


class DefaultMultiNEATCPGClinic(BirthClinic):

    def __init__(self):
        body_genome: Representation = GrammarRepresentation()
        brain_genome: Representation = MultiNEATRepresentation()
        super().__init__(body_genome, brain_genome)


class DefaultLSystemCPGClinic(BirthClinic):

    def __init__(self):
        body_genome: Representation = GrammarRepresentation()
        brain_genome: Representation = MultiNEATRepresentation()
        super().__init__(body_genome, brain_genome)
