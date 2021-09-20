from revolve2.nca.core.actor.individual import Individual
from revolve2.nca.core.genome.genotype import Genotype
from revolve2.revolve.robot.brain.brain import Brain


class Agent(Individual):

    def __init__(self, genotype: Genotype, brain: Brain = None):
        super().__init__(genotype)
        self.brain: Brain = brain
        self.brain.update(genotype.get_random_representation())

    @staticmethod
    def create(individual: Individual, genotype: Genotype, brain: Brain):
        agent = Agent(genotype, brain)
        agent.id = individual.id
        return agent
