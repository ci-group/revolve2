from nca.core.genome import Representation
from nca.revolve.robot.morphology.brain.brain import Brain
from nca.revolve.robot.robot_builder import RobotBuilder


class BrainBuilder(RobotBuilder):
    def __init__(self, representation: type(Representation)):
        super().__init__(representation)

    def build(self) -> Brain:
        pass
