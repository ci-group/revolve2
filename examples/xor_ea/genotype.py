from __future__ import annotations

import config
import numpy as np
from base import Base
from revolve2.core.database import HasId
from revolve2.core.optimization.ea.parameters import Parameters as GenericParameters


class Genotype(Base, HasId, GenericParameters):
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

        return Genotype(tuple(p for p in rng.random(size=config.NUM_PARAMETERS)))

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
            tuple(
                float(v)
                for v in (
                    rng.normal(scale=config.MUTATE_STD, size=config.NUM_PARAMETERS)
                    + self.parameters
                )
            )
        )

    @classmethod
    def crossover(
        cls,
        parent1: Genotype,
        parent2: Genotype,
        rng: np.random.Generator,
    ) -> Genotype:
        """
        Perform crossover between two genotypes.

        :param parent1: The first genotype.
        :param parent2: The second genotype.
        :param rng: Random number generator.
        :returns: A newly created genotype.
        """
        return Genotype(
            tuple(
                b1 if rng.random() < 0.5 else b2
                for b1, b2 in zip(parent1.parameters, parent2.parameters)
            )
        )
