from nca.core.genome.representation import Representation
from revolve.robot.body.body import Body
from revolve.robot.robot_builder import RobotBuilder


class BodyBuilder(RobotBuilder):
    def __init__(self, representation: type(Representation)):
        super().__init__(representation)

    def build(self) -> Body:
        return Body(self.representation)
