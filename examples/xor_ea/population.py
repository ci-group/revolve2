"""Population class."""

from base import Base
from individual import Individual
from revolve2.core.optimization.ea import Population as GenericPopulation


class Population(Base, GenericPopulation[Individual]):
    """SQLAlchemy model for a population."""

    __tablename__ = "population"
