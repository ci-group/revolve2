from nca.core.genome.representation import Representation
from revolve.robot.body.body import Body
from revolve.robot.body.robogen.robogen_grammar import RobogenGrammar
from revolve.robot.body.robogen.robogen_representation import RobogenRepresentation
from revolve.robot.robot_builder import RobotBuilder


class BodyBuilder(RobotBuilder):
    def __init__(self, representation_type: type(Representation)):
        super().__init__(representation_type)

    def build(self) -> Body:

        if self.representation == RobogenRepresentation:
            grammar: RobogenGrammar = RobogenGrammar()
            return Body(self.representation(grammar))

        return Body(self.representation())

