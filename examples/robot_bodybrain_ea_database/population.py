"""Population class."""

from revolve2.experimentation.optimization.ea import Population as GenericPopulation

from .base import Base
from .individual import Individual


class Population(Base, GenericPopulation[Individual]):
    """A population of individuals."""

    __tablename__ = "population"
