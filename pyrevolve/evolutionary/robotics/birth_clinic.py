from pyrevolve.evolutionary import Fitness
from pyrevolve.evolutionary.agents import Agents
from pyrevolve.evolutionary.algorithm.genome.representation import Representation
from pyrevolve.evolutionary.algorithm.genome.representations.direct_representation import GrammarRepresentation
from pyrevolve.evolutionary.algorithm.genome.representations.l_system.lsystem_representation import \
    LSystemRepresentation
from pyrevolve.evolutionary.robotics.morphology.brain.representation.multineat_representation import MultiNEATRepresentation
from pyrevolve.evolutionary.robotics.morphology.body.body_builder import BodyBuilder
from pyrevolve.evolutionary.robotics.morphology.brain.brain_builder import BrainBuilder
from pyrevolve.evolutionary.robotics.robot import Robot
from pyrevolve.shared.configurations import RobotConfiguration


class BirthClinic:

    def __init__(self, body_genome: BodyBuilder = BodyBuilder(Representation),
                 brain_genome: BrainBuilder = BrainBuilder(Representation),
                 fitness: type(Fitness) = Fitness):
        self.configuration = RobotConfiguration()

        self.body_builder: BodyBuilder = body_genome
        self.brain_builder: BrainBuilder = brain_genome
        self.fitness_type: type(Fitness) = fitness

    def create(self) -> Agents:
        robots: Agents = Agents()

        for robot_index in range(self.configuration.number_of_robots):
            robots.add(self.build_robot())

        return robots

    def build_robot(self) -> Robot:
        return Robot(self.fitness_type(), self.body_builder.build(), self.brain_builder.build())


class DefaultMultiNEATCPGClinic(BirthClinic):

    def __init__(self):
        super().__init__(BodyBuilder(GrammarRepresentation), BrainBuilder(MultiNEATRepresentation))


class DefaultLSystemCPGClinic(BirthClinic):

    def __init__(self):
        super().__init__(BodyBuilder(LSystemRepresentation), BrainBuilder(MultiNEATRepresentation))
