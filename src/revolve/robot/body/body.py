from nca.core.genome.representation import Representation
from revolve.robot.morphology import Morphology


class Body(Morphology):

    def __init__(self, genome: Representation):
        super().__init__(genome)

