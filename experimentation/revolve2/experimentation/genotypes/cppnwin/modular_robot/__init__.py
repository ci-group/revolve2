"""CPPNWIN genotypes for modular robots."""

from revolve2.experimentation.genotypes.cppnwin.modular_robot.v1._body_genotype_orm import (
    BodyGenotypeOrmV1,
)

from .v1 import BodyGenotypeV1
from .v2 import BodyGenotypeV2

from ._brain_cpg_network_neighbor_v1 import BrainCpgNetworkNeighborV1
from ._brain_genotype_cpg import BrainGenotypeCpg
from ._brain_genotype_cpg_orm import BrainGenotypeCpgOrm

__all__ = [
    "BodyGenotypeV1",
    "BodyGenotypeV2",
    "BodyGenotypeOrmV1",
    "BrainCpgNetworkNeighborV1",
    "BrainGenotypeCpg",
    "BrainGenotypeCpgOrm",
]
