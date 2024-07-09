"""Experiment class."""

import sqlalchemy.orm as orm
from revolve2.experimentation.database import HasId

from ._base import Base


class Experiment(Base, HasId):
    """Experiment description."""

    __tablename__ = "experiment"

    # The seed for the rng.
    rng_seed: orm.Mapped[int] = orm.mapped_column(nullable=False)
