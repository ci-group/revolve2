from nca.core.actor.fitness import Fitness
from nca.core.actor.individual import Actor
from revolve.robot.body.body import Body
from revolve.robot.brain.brain import Brain


class Robot(Actor):

    def __init__(self, actor: Actor):
        self.actor = actor
        self.brain: Brain = None
        self.body: Body = None
        self.measures = None

    def performance(self, fitness: Fitness):
        self.actor.performance(fitness)
