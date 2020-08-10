from nca.core.genome.representation import Representation
from revolve.robot.brain.brain import Brain
from revolve.robot.robot_builder import RobotBuilder


class BrainBuilder(RobotBuilder):
    def __init__(self, representation: type(Representation)):
        super().__init__(representation)

    def build(self) -> Brain:
        return Brain(self.representation)
