from nca.core.genome import Representation
from nca.revolve.robot.morphology.morphology import Morphology


class Body(Morphology):

    def __init__(self, genome: Representation):
        super().__init__(genome)
