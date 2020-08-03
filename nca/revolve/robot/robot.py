from nca.core.agent import Individual
from nca.core.agent import Fitness
from nca.revolve.robot.morphology.body.body import Body
from nca.revolve.robot.morphology.brain.brain import Brain
from nca.experiment.sequential_identifier import SequentialIdentifier


class Robot(Individual):
    robot_identifier = SequentialIdentifier()

    def __init__(self, fitness: Fitness, body: Body = None, brain: Brain = None):
        super().__init__(self.robot_identifier.index(), fitness)

        self.brain = brain
        self.body = body
