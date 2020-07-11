from pyrevolve.evolutionary.algorithm.genome.representation import Representation
from pyrevolve.evolutionary.robotics.morphology.morphology import Morphology


class Body(Morphology):

    def __init__(self, genome: Representation):
        super().__init__(genome)
