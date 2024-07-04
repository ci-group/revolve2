"""Individual class."""

from dataclasses import dataclass

from .base import Base
from .genotype import Genotype
import sqlalchemy.orm as orm

from revolve2.experimentation.optimization.ea import Individual as GenericIndividual


@dataclass
class Individual(
    Base, GenericIndividual[Genotype], population_table="population", kw_only=True
):
    """An individual in a population."""

    age: orm.Mapped[int] = orm.mapped_column(nullable=True)
    novelty: orm.Mapped[float] = orm.mapped_column(nullable=True)
    __tablename__ = "individual"
