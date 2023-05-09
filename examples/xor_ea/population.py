from base import Base
from individual import Individual
from revolve2.core.optimization.ea.population import Population as GenericPopulation


class Population(Base, GenericPopulation[Individual]):
    __tablename__ = "population"
