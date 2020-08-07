from nca.core.agent.individual import Individual
from nca.core.agent.fitness import Fitness
from revolve.robot.body.body import Body
from revolve.robot.brain.brain import Brain
from nca.core.abstract.sequential_identifier import SequentialIdentifier


class Robot(Individual):
    robot_identifier = SequentialIdentifier()

    def __init__(self, body: Body = None, brain: Brain = None, fitness: Fitness = None):
        super().__init__(self.robot_identifier.id(), fitness)

        self.brain = brain
        self.body = body
