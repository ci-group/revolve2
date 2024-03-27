"""Population class."""

import sqlalchemy.ext.orderinglist
import sqlalchemy.orm as orm

from revolve2.experimentation.database import HasId

from ._base import Base
from ._individual import Individual


class Population(Base, HasId, kw_only=True):
    """
    A population of individuals.

    Contain a list of individuals.
    First take a look at the Individual class.
    SQLAlchemy automatically uses the individuals 'population_id' and 'population_index' to
    create a table of individuals referring to populations, while retaining the individuals original order in the population.
    """

    __tablename__ = "population"

    individuals: orm.Mapped[list[Individual]] = orm.relationship(
        order_by=Individual.population_index,
        collection_class=sqlalchemy.ext.orderinglist.ordering_list("population_index"),
    )
