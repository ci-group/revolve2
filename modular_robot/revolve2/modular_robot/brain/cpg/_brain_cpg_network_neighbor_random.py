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

    def __init__(
        self, body: Body, rng: np.random.Generator, passive_connections: bool
    ) -> None:
        """
        Initialize this object.

        :param body: The body to create the cpg network and brain for.
        :param rng: Random number generator used for generating the weights.
        :param passive_connections: Wether to add CPGs also to passive body modules. The passive module CPGs will
        not produce output and serve more as communication bridges.
        """
        self._rng = rng
        super().__init__(body, passive_connections)

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
