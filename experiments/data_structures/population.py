"""Population class."""

from .base import Base
from .individual import Individual

from revolve2.experimentation.optimization.ea import Population as GenericPopulation


class Population(Base, GenericPopulation[Individual], kw_only=True):
    """A population of individuals."""

    __tablename__ = "population"
