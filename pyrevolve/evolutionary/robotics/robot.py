from pyrevolve.evolutionary.individual import Agent
from pyrevolve.evolutionary.fitness import Fitness
from pyrevolve.evolutionary.robotics.morphology.body.body import Body
from pyrevolve.evolutionary.robotics.morphology.brain.brain import Brain
from pyrevolve.shared.sequential_identifier import SequentialIdentifier


class Robot(Agent):
    robot_identifier = SequentialIdentifier()

    def __init__(self, fitness: Fitness, body: Body = None, brain: Brain = None):
        super().__init__(self.robot_identifier.index(), fitness)

        self.brain = brain
        self.body = body
