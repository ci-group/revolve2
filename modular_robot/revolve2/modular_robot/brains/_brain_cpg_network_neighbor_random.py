import numpy as np

from .. import Body
from .._common import ActiveHinge
from ._brain_cpg_network_neighbor import BrainCpgNetworkNeighbor


class BrainCpgNetworkNeighborRandom(BrainCpgNetworkNeighbor):
    """A cpg brain with random weights between neurons."""

    _rng: np.random.Generator

    def __init__(self, rng: np.random.Generator) -> None:
        """
        Initialize this object.

        :param rng: Random number generator used for generating the weights.
        """
        self._rng = rng

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
