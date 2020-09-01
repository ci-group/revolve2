from nca.core.actor.fitness import Fitness
from nca.core.actor.individual import Individual, Actor
from revolve.robot.brain.brain import Brain


class Agent(Actor):

    def __init__(self, individual: Individual, brain: Brain = None):
        self.individual = individual
        self.brain: Brain = brain

    def performance(self, fitness: Fitness):
        self.individual.performance()
