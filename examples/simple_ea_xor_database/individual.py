"""Individual class."""

from dataclasses import dataclass

from genotype import Genotype
from base import Base
import sqlalchemy.orm as orm
from revolve2.core.database import HasId
import sqlalchemy


@dataclass
class Individual(Base, HasId):
    """
    An individual in a population.

    Stores a reference to its population.
    In SQLAlchemy this can be done with an 'id' field, which refers to the id of the population here.
    We also defined 'population_index', which is used to retain the order of the individuals in the population.
    Both are used in the Population class to create a proper array in the database.

    Every individual has a corresponding genotype.
    The 'genotype' field contain a reference to the Genotype instance.
    'genotype_id' is used in the database; it contain the id of the genotype of.
    SQLAlchemy automatically links these two together, so a user only has to ever work with the genotype field.

    Finally, 'fitness' is simply the fitness of the individual.
    """

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
