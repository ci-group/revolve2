"""Genotype class."""

from __future__ import annotations

from dataclasses import dataclass

import multineat
import numpy as np
from revolve2.ci_group.genotypes.cppnwin.modular_robot import BrainGenotypeCpg
from revolve2.ci_group.genotypes.cppnwin.modular_robot.v2 import BodyGenotypeV2
from revolve2.modular_robot import ModularRobot


@dataclass
class Genotype(BodyGenotypeV2, BrainGenotypeCpg):
    """A genotype for a body and brain using CPPN."""

    @classmethod
    def random(
        cls,
        innov_db_body: multineat.InnovationDatabase,
        innov_db_brain: multineat.InnovationDatabase,
        rng: np.random.Generator,
    ) -> Genotype:
        """
        Create a random genotype.

        :param innov_db_body: Multineat innovation database for the body. See Multineat library.
        :param innov_db_brain: Multineat innovation database for the brain. See Multineat library.
        :param rng: Random number generator.
        :returns: The created genotype.
        """
        body = cls.random_body(innov_db_body, rng)
        brain = cls.random_brain(innov_db_brain, rng)

        return Genotype(body=body.body, brain=brain.brain)

    def mutate(
        self,
        innov_db_body: multineat.InnovationDatabase,
        innov_db_brain: multineat.InnovationDatabase,
        rng: np.random.Generator,
    ) -> Genotype:
        """
        Mutate this genotype.

        This genotype will not be changed; a mutated copy will be returned.

        :param innov_db_body: Multineat innovation database for the body. See Multineat library.
        :param innov_db_brain: Multineat innovation database for the brain. See Multineat library.
        :param rng: Random number generator.
        :returns: A mutated copy of the provided genotype.
        """
        body = self.mutate_body(innov_db_body, rng)
        brain = self.mutate_brain(innov_db_brain, rng)

        return Genotype(body=body.body, brain=brain.brain)

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
        body = cls.crossover_body(parent1, parent2, rng)
        brain = cls.crossover_brain(parent1, parent2, rng)

        return Genotype(body=body.body, brain=brain.brain)

    def develop(self) -> ModularRobot:
        """
        Develop the genotype into a modular robot.

        :returns: The created robot.
        """
        body = self.develop_body()
        brain = self.develop_brain(body=body)
        return ModularRobot(body=body, brain=brain)
