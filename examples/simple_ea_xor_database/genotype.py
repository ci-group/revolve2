"""Genotype class."""

from __future__ import annotations

import config
import numpy as np
from revolve2.core.optimization.ea import Parameters as GenericParameters
from revolve2.core.database import HasId
from base import Base


class Genotype(Base, HasId, GenericParameters):
    """
    ORM definition for our genotype that is a list of parameters.

    In SQLAlchemy we can inherit from multiple classes that each define seperate table columns.
    Revolve2's 'GenericParameters' class defines our 'parameters' field.
    Take a short look at the class to see that it writes the parameters to the database as a string of semicolon concatened floats.

    Besides the removed dataclass decorator(identical behavior is provided by SQLAlchemy),
    the rest of this class is exactly the same as the original example!
    """

    __tablename__ = "genotype"

    @classmethod
    def random(
        cls,
        rng: np.random.Generator,
    ) -> Genotype:
        """
        Create a random genotype.

        :param rng: Random number generator.
        :returns: The created genotype.
        """
        return Genotype(rng.random(size=config.NUM_PARAMETERS) * 2 - 1)

    def mutate(
        self,
        rng: np.random.Generator,
    ) -> Genotype:
        """
        Mutate this genotype.

        This genotype will not be changed; a mutated copy will be returned.

        :param rng: Random number generator.
        :returns: A mutated copy of the provided genotype.
        """
        return Genotype(
            rng.normal(scale=config.MUTATE_STD, size=config.NUM_PARAMETERS)
            + self.parameters
        )

    @classmethod
    def crossover(
        cls,
        parent1: Genotype,
        parent2: Genotype,
        rng: np.random.Generator,
    ) -> Genotype:
        """
        Perform uniform crossover between two genotypes.

        :param parent1: The first genotype.
        :param parent2: The second genotype.
        :param rng: Random number generator.
        :returns: A newly created genotype.
        """
        mask = rng.random(config.NUM_PARAMETERS)
        return Genotype(np.where(mask, parent1.parameters, parent2.parameters))
