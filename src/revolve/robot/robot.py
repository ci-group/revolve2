from nca.core.actor.individual import Individual
from nca.core.genome.genotype import Genotype
from revolve.robot.body.body import Body
from revolve.robot.brain.brain import Brain


class Robot(Individual):

    def __init__(self, genotype: Genotype):
        super().__init__(genotype)

        self.brain: Brain = None
        self.body: Body = None

        self.measures = None

