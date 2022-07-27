from random import Random
from typing import List, Tuple

from revolve2.core.modular_robot import ActiveHinge, Body

from ._brain_cpg_network_neighbour import BrainCpgNetworkNeighbour


class BrainCpgNetworkNeighbourRandom(BrainCpgNetworkNeighbour):
    """A cpg brain with random weights between neurons."""

    _rng: Random

    def __init__(self, rng: Random) -> None:
        """
        Initialize this object.

        :param rng: Random number generator used for generating the weights.
        """
        self._rng = rng

    def _make_weights(
        self,
        active_hinges: List[ActiveHinge],
        connections: List[Tuple[ActiveHinge, ActiveHinge]],
        body: Body,
    ) -> Tuple[List[float], List[float]]:
        return (
            [self._rng.random() * 2.0 - 1 for _ in range(len(active_hinges))],
            [self._rng.random() * 2.0 - 1 for _ in range(len(connections))],
        )
