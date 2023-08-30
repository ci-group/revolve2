"""Individual class."""

from dataclasses import dataclass

import sqlalchemy
import sqlalchemy.orm as orm
from base import Base
from genotype import Genotype
from revolve2.core.database import HasId


@dataclass
class Individual(Base, HasId):
    """An individual in a population."""

    __tablename__ = "individual"

    population_id: orm.Mapped[int] = orm.mapped_column(
        sqlalchemy.ForeignKey("population.id"), nullable=False, init=False
    )
    population_index: orm.Mapped[int] = orm.mapped_column(nullable=False, init=False)
    genotype_id: orm.Mapped[int] = orm.mapped_column(
        sqlalchemy.ForeignKey("genotype.id"), nullable=False, init=False
    )
    genotype: orm.Mapped[Genotype] = orm.relationship()
    fitness: orm.Mapped[float] = orm.mapped_column(nullable=False)
