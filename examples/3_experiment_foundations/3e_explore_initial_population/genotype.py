"""Genotype class."""

from __future__ import annotations

from dataclasses import dataclass

import multineat
import numpy as np

from revolve2.modular_robot import ModularRobot
from revolve2.standards.genotypes.cppnwin.modular_robot import BrainGenotypeCpg
from revolve2.standards.genotypes.cppnwin.modular_robot.v2 import BodyGenotypeV2


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

    def develop(self, visualize: bool = False) -> ModularRobot:
        """
        Develop the genotype into a modular robot.

        :param visualize: Wether to plot the mapping from genotype to phenotype.
        :returns: The created robot.
        """
        body = self.develop_body(visualize=visualize)
        brain = self.develop_brain(body=body)
        return ModularRobot(body=body, brain=brain)
