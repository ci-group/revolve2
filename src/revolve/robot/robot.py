
from nca.core.agent.fitness import Fitness
from nca.core.agent.individual import Individual
from revolve.robot.body.body import Body
from revolve.robot.brain.brain import Brain
from revolve.robot.morphology import MorphologyType


class Agent(Individual):
    def __init__(self, brain: Brain, fitness: Fitness):
        super().__init__({MorphologyType.BRAIN: brain}, fitness)


class Robot(Individual):

    def __init__(self, body: Body = None, brain: Brain = None, fitness: Fitness = None):
        super().__init__({MorphologyType.BRAIN: brain, MorphologyType.BODY: body}, fitness)
