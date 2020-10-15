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
        self.actor.fitness.add(fitness(self))
        self.actor.fitness = self.actor.fitness()  # calculate final/intermediate average

