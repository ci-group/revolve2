from pyrevolve.evolutionary.algorithm.genome.representation import Representation
from pyrevolve.evolutionary.robotics.morphology.brain.brain import Brain
from pyrevolve.evolutionary.robotics.robot_builder import RobotBuilder


class BrainBuilder(RobotBuilder):
    def __init__(self, representation: type(Representation)):
        super().__init__(representation)

    def build(self) -> Brain:
        pass
