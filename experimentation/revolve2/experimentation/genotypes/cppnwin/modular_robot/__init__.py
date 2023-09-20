"""CPPNWIN genotypes for modular robots."""

from revolve2.experimentation.genotypes.cppnwin.modular_robot._body_genotype_orm import (
    BodyGenotypeOrm,
)

from ._body_genotype import BodyGenotype
from ._brain_cpg_network_neighbor_v1 import BrainCpgNetworkNeighborV1
from ._brain_genotype_cpg import BrainGenotypeCpg
from ._brain_genotype_cpg_orm import BrainGenotypeCpgOrm

__all__ = [
    "BodyGenotype",
    "BodyGenotypeOrm",
    "BrainCpgNetworkNeighborV1",
    "BrainGenotypeCpg",
    "BrainGenotypeCpgOrm",
]
