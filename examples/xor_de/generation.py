from base import Base
from population import Population
from revolve2.core.optimization.ea.generation import Generation as GenericGeneration


class Generation(Base, GenericGeneration[Population]):
    __tablename__ = "generation"
