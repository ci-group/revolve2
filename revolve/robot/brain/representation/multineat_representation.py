from nca.core.genome.representation import Representation


class MultiNEATRepresentation(Representation):

    def __init__(self):
        super().__init__()

    def _initialize(self):
        pass

    def compatibility(self, other) -> float:
        pass