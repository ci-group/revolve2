"""Brain for modular robots."""

from ._brain_cpg_network_neighbor import BrainCpgNetworkNeighbor
from ._brain_cpg_network_neighbor_random import BrainCpgNetworkNeighborRandom
from ._brain_cpg_network_static import BrainCpgNetworkStatic
from ._make_cpg_network_structure_neighbor import (
    active_hinges_to_cpg_network_structure_neighbor,
    body_to_actor_and_cpg_network_structure_neighbour,
)

__all__ = [
    "BrainCpgNetworkNeighbor",
    "BrainCpgNetworkNeighborRandom",
    "BrainCpgNetworkStatic",
    "active_hinges_to_cpg_network_structure_neighbor",
    "body_to_actor_and_cpg_network_structure_neighbour",
]
