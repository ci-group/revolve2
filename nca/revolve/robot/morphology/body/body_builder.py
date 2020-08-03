from nca.core.genome import Representation
from nca.revolve.robot import Body
from nca.revolve.robot.robot_builder import RobotBuilder


class BodyBuilder(RobotBuilder):
    def __init__(self, representation: type(Representation)):
        super().__init__(representation)

    def build(self) -> Body:
        pass

