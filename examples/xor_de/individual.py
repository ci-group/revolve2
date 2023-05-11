from base import Base
from genotype import Genotype
from revolve2.core.optimization.ea.individual import Individual as GenericIndividual


class Individual(Base, GenericIndividual[Genotype], population_table="population"):
    __tablename__ = "individual"
