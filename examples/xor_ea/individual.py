"""Individual class."""

from base import Base
from genotype import Genotype
from revolve2.core.optimization.ea import Individual as GenericIndividual


class Individual(Base, GenericIndividual[Genotype], population_table="population"):
    """SQLAlchemy model of an individual."""

    __tablename__ = "individual"
