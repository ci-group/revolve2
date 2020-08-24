from nca.core.abstract.creational.builder import Builder
from nca.core.agent.individual import Individual
from nca.core.agent.agents import Agents
from nca.core.agent.fitness import Fitness
from nca.core.agent.individual_factory import ActorFactory
from nca.core.evolution.conditions.initialization import Initialization
from nca.core.genome.representation import Representation
from revolve.robot.body.body import Body, BodyRepresentation
from revolve.robot.body.body_builder import BodyBuilder
from revolve.robot.brain.brain import Brain, BrainRepresentation
from revolve.robot.morphology import MorphologyType
from src.revolve.robot.brain.brain_builder import BrainBuilder

from revolve.robot.robot import Robot


class RobotFactory(ActorFactory):

    def __init__(self, body_genome: type(BodyRepresentation), brain_genome: type(BrainRepresentation)):
        super().__init__()
        self.body_genome_type: type(BodyRepresentation) = body_genome
        self.brain_genome_type: type(BrainRepresentation) = brain_genome
        self.body_initialization: Initialization = Initialization()
        self.brain_initialization: Initialization = Initialization()

    def initialize(self, initialization: Initialization = Initialization(), fitness_type: type(Fitness) = Fitness):
        self.initialization = initialization
        self.fitness_type = fitness_type
        return self

    def create(self, number_of_robots: int) -> Agents:
        agents: Agents = Agents()
        # TODO raise when body / brain initialization does not exist.

        for robot_index in range(number_of_robots):
            agents.add(Robot(Body(self.body_genome_type(self.body_initialization)),
                             Brain(self.brain_genome_type(self.brain_initialization))))

        return agents


class BirthClinic(Builder):

    def __init__(self, body_builder: BodyBuilder = BodyBuilder(Representation),
                 brain_builder: BrainBuilder = BrainBuilder(Representation)):
        super().__init__()
        self.body_builder: BodyBuilder = body_builder
        self.brain_builder: BrainBuilder = brain_builder

    def build(self, robot: Robot) -> Individual:
        robot.representation[MorphologyType.BODY].develop()
        robot.representation[MorphologyType.BRAIN].develop()
        return robot
