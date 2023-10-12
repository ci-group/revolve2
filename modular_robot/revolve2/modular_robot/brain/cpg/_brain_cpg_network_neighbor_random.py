import numpy as np

from ...body.base import ActiveHinge, Body
from ._brain_cpg_network_neighbor import BrainCpgNetworkNeighbor


class BrainCpgNetworkNeighborRandom(BrainCpgNetworkNeighbor):
    """
    A cpg brain with random weights between neurons.

    The weights are randomly generated when this object is created,
    so they will be the same for every controller instance.
    """

    _rng: np.random.Generator

    def __init__(self, body: Body, rng: np.random.Generator) -> None:
        """
        Initialize this object.

        :param body: The body to create the cpg network and brain for.
        :param rng: Random number generator used for generating the weights.
        """
        self._rng = rng
        super().__init__(body)

    def _make_weights(
        self,
        active_hinges: list[ActiveHinge],
        connections: list[tuple[ActiveHinge, ActiveHinge]],
        body: Body,
    ) -> tuple[list[float], list[float]]:
        return (
            (self._rng.random(size=len(active_hinges)) * 2.0 - 1).tolist(),
            (self._rng.random(size=len(connections)) * 2.0 - 1).tolist(),
        )
