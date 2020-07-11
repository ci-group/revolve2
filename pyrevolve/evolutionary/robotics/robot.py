from pyrevolve.evolutionary.agent import Agent
from pyrevolve.evolutionary.fitness import Fitness
from pyrevolve.evolutionary.robotics.body.robot_body import RobotBody
from pyrevolve.evolutionary.robotics.brain.robot_brain import RobotBrain


class Robot(Agent):

    def __init__(self, robot_id: int, fitness: Fitness,
                 brain: RobotBrain = None, body: RobotBody = None):
        super().__init__(robot_id, fitness)

        self.brain = brain
        self.body = body
