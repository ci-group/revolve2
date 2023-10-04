"""Individual class."""

from dataclasses import dataclass

from revolve2.experimentation.optimization.ea import Individual as GenericIndividual

from .base import Base
from .genotype import Genotype


@dataclass
class Individual(Base, GenericIndividual[Genotype], population_table="population"):
    """An individual in a population."""

    __tablename__ = "individual"
