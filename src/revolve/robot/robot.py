from nca.core.abstract.sequential_identifier import SequentialIdentifier
from nca.core.agent.fitness import Fitness
from nca.core.agent.individual import Individual
from revolve.robot.body.body import Body
from revolve.robot.brain.brain import Brain


class Robot(Individual):
    robot_identifier = SequentialIdentifier()

    def __init__(self, body: Body = None, brain: Brain = None, fitness: Fitness = None):
        super().__init__(self.robot_identifier.id(), fitness)

        self.brain = brain
        self.body = body
