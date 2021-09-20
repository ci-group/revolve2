from revolve2.nca.core.actor.individual import Individual
from revolve2.revolve.robot.body.body import Body
from revolve2.revolve.robot.brain.brain import Brain


class Robot(Individual):

    def __init__(self, genotype, brain: Brain = None, body: Body = None):
        super().__init__(genotype)

        self.brain: Brain = brain
        self.body: Body = body
        
        self.measures = None
