from pyrevolve.evolutionary.individual import Agent
from pyrevolve.evolutionary.fitness import Fitness
from pyrevolve.evolutionary.robotics.morphology.body.body import Body
from pyrevolve.evolutionary.robotics.morphology.brain.brain import Brain


class Robot(Agent):

    def __init__(self, robot_id: int, fitness: Fitness,
                 body: Body = None, brain: Brain = None):
        super().__init__(robot_id, fitness)

        self.brain = brain
        self.body = body
