"""Brain for modular robots."""

from ._brain_cpg_network_neighbor import BrainCpgNetworkNeighbor
from ._brain_cpg_network_neighbor_random import BrainCpgNetworkNeighborRandom
from ._brain_cpg_network_static import BrainCpgNetworkStatic
from ._make_cpg_network_structure_neighbor import make_cpg_network_structure_neighbor

__all__ = [
    "BrainCpgNetworkNeighbor",
    "BrainCpgNetworkNeighborRandom",
    "BrainCpgNetworkStatic",
    "make_cpg_network_structure_neighbor",
]
