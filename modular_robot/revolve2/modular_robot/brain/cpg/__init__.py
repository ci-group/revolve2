"""Cpg brains for modular robots."""

from ._brain_cpg_instance import BrainCpgInstance
from ._brain_cpg_network_neighbor import BrainCpgNetworkNeighbor
from ._brain_cpg_network_neighbor_random import BrainCpgNetworkNeighborRandom
from ._brain_cpg_network_static import BrainCpgNetworkStatic
from ._cpg_network_structure import CpgNetworkStructure
from ._brain_cpg_instance_environmental_control import BrainCpgInstanceEnvironmentalControl
from ._brain_cpg_network_static_environmental_control import BrainCpgNSEnvironmentalControl
from ._make_cpg_network_structure_neighbor import (
    active_hinges_to_cpg_network_structure_neighbor,
)

__all__ = [
    "BrainCpgInstance",
    "BrainCpgNetworkNeighbor",
    "BrainCpgNetworkNeighborRandom",
    "BrainCpgNetworkStatic",
    "BrainCpgInstanceEnvironmentalControl",
    "BrainCpgNSEnvironmentalControl",
    "CpgNetworkStructure",
    "active_hinges_to_cpg_network_structure_neighbor",
]
