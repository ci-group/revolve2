from revolve.robot.body.body import Body
from revolve.robot.body.robogen.robogen_representation import RobogenRepresentation


class RobogenBody(Body):

    def __init__(self, representation: RobogenRepresentation):
        super().__init__(representation)
