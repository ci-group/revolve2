"""Individual class."""

from dataclasses import dataclass

from revolve2.experimentation.optimization.ea import Individual as GenericIndividual

from ._base import Base
from ._genotype import Genotype


@dataclass
class Individual(
    Base, GenericIndividual[Genotype], population_table="population", kw_only=True
):
    """An individual in a population."""

    __tablename__ = "individual"
