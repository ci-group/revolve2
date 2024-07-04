"""Experiment class."""

import sqlalchemy.orm as orm
from revolve2.experimentation.database import HasId

from ._base import Base


class Experiment(Base, HasId):
    """
    This is the ORM class that describes an experiment.

    Every experiment run will create an instance of this class and save it to the database.
    All other object in the database will directly or indirectly refer to an instance of this class,
    so you always know which experiment they are part of.
    Additionally it is a great place to store experiment parameters, as well your generated rng seed.

    Next to defining our own or 'rng_seed' field,
    we inherit from the Revolve2 'HasId' class which defines an 'id' variable for us.
    """

    # We have to defined the name of this classes table in the database.
    __tablename__ = "experiment"

    # The seed for the rng.
    # Very important to save (for reproducibility) as it is generated at the start of the program based on the current time.
    rng_seed: orm.Mapped[int] = orm.mapped_column(nullable=False)
