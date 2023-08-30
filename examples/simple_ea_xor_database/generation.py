"""Generation class."""

from base import Base
from population import Population
import sqlalchemy.orm as orm
import sqlalchemy
from revolve2.core.database import HasId
from experiment import Experiment


class Generation(Base, HasId):
    """
    A single finished iteration of the EA.

    We reference the experiment so we know which experiment this generation is part of.
    In addition every generation has a corresponding population.
    This is all done in a similar way to the genotype, individual, and population classes.

    Finally, we save the number of the current generation.
    """

    __tablename__ = "generation"

    experiment_id: orm.Mapped[int] = orm.mapped_column(
        sqlalchemy.ForeignKey("experiment.id"), nullable=False, init=False
    )
    experiment: orm.Mapped[Experiment] = orm.relationship()
    generation_index: orm.Mapped[int] = orm.mapped_column(nullable=False)
    population_id: orm.Mapped[int] = orm.mapped_column(
        sqlalchemy.ForeignKey("population.id"), nullable=False, init=False
    )
    population: orm.Mapped[Population] = orm.relationship()
