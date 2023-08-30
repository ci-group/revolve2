"""Population class."""

import sqlalchemy.ext.orderinglist
import sqlalchemy.orm as orm
from base import Base
from individual import Individual
from revolve2.core.database import HasId


class Population(Base, HasId):
    """A population of individuals."""

    __tablename__ = "population"

    individuals: orm.Mapped[list[Individual]] = orm.relationship(
        order_by=Individual.population_index,
        collection_class=sqlalchemy.ext.orderinglist.ordering_list("population_index"),
    )
