from nca.core.genome.representations.representation import Representation


class NetworkRepresentation(Representation):
    pass


class MultiNEATRepresentation(NetworkRepresentation):

    def __init__(self):
        super().__init__()

    def _initialize(self):
        pass

    def compatibility(self, other) -> float:
        pass
