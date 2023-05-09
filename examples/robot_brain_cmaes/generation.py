from typing import (
    List,
    TypeVar,
)

import sqlalchemy.ext.orderinglist
import sqlalchemy.orm as orm
from revolve2.core.database import HasId
from base import Base
from parameters import Parameters

TIndividual = TypeVar("TIndividual")


class Generation(Base, HasId):
    __tablename__ = "generation"

    generation_index: orm.Mapped[int] = orm.mapped_column(nullable=False, unique=True)

    individuals: orm.Mapped[List[Parameters]] = orm.relationship(
        order_by=Parameters.generation_index,
        collection_class=sqlalchemy.ext.orderinglist.ordering_list("generation_index"),
    )
