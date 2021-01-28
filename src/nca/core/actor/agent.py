from nca.core.actor.individual import Individual
from nca.core.genome.genotype import Genotype
from revolve.robot.brain.brain import Brain


class Agent(Individual):

    def __init__(self, genotype: Genotype, brain: Brain = None):
        super().__init__(genotype)
        self.brain: Brain = brain
        self.brain.update(genotype.get_random_representation())
