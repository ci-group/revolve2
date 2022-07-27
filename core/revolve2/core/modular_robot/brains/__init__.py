"""Brain for modular robots."""

from ._brain_cpg_network_neighbour import BrainCpgNetworkNeighbour
from ._brain_cpg_network_neighbour_random import BrainCpgNetworkNeighbourRandom
from ._brain_cpg_network_static import BrainCpgNetworkStatic
from ._make_cpg_network_structure_neighbour import make_cpg_network_structure_neighbour

__all__ = [
    "BrainCpgNetworkNeighbour",
    "BrainCpgNetworkNeighbourRandom",
    "BrainCpgNetworkStatic",
    "make_cpg_network_structure_neighbour",
]
