"""CPPNWIN genotypes for modular robots."""

from ._brain_cpg_network_neighbor_v1 import BrainCpgNetworkNeighborV1
from ._brain_genotype_cpg import BrainGenotypeCpg
from ._brain_genotype_cpg_orm import BrainGenotypeCpgOrm

__all__ = [
    "BrainCpgNetworkNeighborV1",
    "BrainGenotypeCpg",
    "BrainGenotypeCpgOrm",
]
