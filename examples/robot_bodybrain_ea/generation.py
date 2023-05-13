"""Generation class."""

from base import Base
from population import Population
from revolve2.core.optimization.ea import Generation as GenericGeneration


class Generation(Base, GenericGeneration[Population]):
    """SQLAlchemy model for a generation."""

    __tablename__ = "generation"
