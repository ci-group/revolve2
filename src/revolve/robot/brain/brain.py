from nca.core.genome.representation import Representation
from revolve.robot.morphology import Morphology


class Brain(Morphology):

    def __init__(self, genome: Representation):
        super().__init__(genome)
