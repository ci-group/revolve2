from nca.core.agent import Fitness
from nca.core.agent.agents import Agents
from nca.core.genome import Representation
from nca.core.genome.representations import GrammarRepresentation
from nca.core.genome.representations import \
    LSystemRepresentation
from nca.revolve.robot import BodyBuilder
from nca.revolve.robot.morphology.brain import BrainBuilder
from nca.revolve.robot.morphology.brain import MultiNEATRepresentation
from nca.revolve.robot import Robot
from nca.experiment.shared import RobotConfiguration


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
