from pyrevolve.evolutionary.algorithm.genome.representation import Representation
from pyrevolve.evolutionary.robotics.morphology.body.body import Body
from pyrevolve.evolutionary.robotics.robot_builder import RobotBuilder


class BodyBuilder(RobotBuilder):
    def __init__(self, representation: type(Representation)):
        super().__init__(representation)

    def build(self) -> Body:
        pass

