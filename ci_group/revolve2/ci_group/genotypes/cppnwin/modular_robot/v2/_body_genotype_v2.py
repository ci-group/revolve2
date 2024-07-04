from __future__ import annotations

from dataclasses import dataclass

import multineat
import numpy as np
from revolve2.modular_robot.body.v2 import BodyV2
from typing_extensions import Self

from ..._multineat_genotype_pickle_wrapper import MultineatGenotypePickleWrapper
from ..._multineat_rng_from_random import multineat_rng_from_random
from ..._random_multineat_genotype import random_multineat_genotype
from .._multineat_params import get_multineat_params
from ._body_develop import develop


@dataclass
class BodyGenotypeV2:
    """CPPNWIN body genotype."""

    _NUM_INITIAL_MUTATIONS = 5
    _MULTINEAT_PARAMS = get_multineat_params()
    body: MultineatGenotypePickleWrapper

    @classmethod
    def random_body(
        cls,
        innov_db: multineat.InnovationDatabase,
        rng: np.random.Generator,
    ) -> BodyGenotypeV2:
        """
        Create a random genotype.

        :param innov_db: Multineat innovation database. See Multineat library.
        :param rng: Random number generator.
        :returns: The created genotype.
        """
        multineat_rng = multineat_rng_from_random(rng)

        body = MultineatGenotypePickleWrapper(
            random_multineat_genotype(
                innov_db=innov_db,
                rng=multineat_rng,
                multineat_params=cls._MULTINEAT_PARAMS,
                output_activation_func=multineat.ActivationFunction.UNSIGNED_SINE,
                num_inputs=5,  # bias(always 1), pos_x, pos_y, pos_z, chain_length
                num_outputs=2,  # block_type, rotation_type
                num_initial_mutations=cls._NUM_INITIAL_MUTATIONS,
            )
        )

        return BodyGenotypeV2(body)

    def mutate_body(
        self,
        innov_db: multineat.InnovationDatabase,
        rng: np.random.Generator,
    ) -> BodyGenotypeV2:
        """
        Mutate this genotype.

        This genotype will not be changed; a mutated copy will be returned.

        :param innov_db: Multineat innovation database. See Multineat library.
        :param rng: Random number generator.
        :returns: A mutated copy of the provided genotype.
        """
        multineat_rng = multineat_rng_from_random(rng)

        return BodyGenotypeV2(
            MultineatGenotypePickleWrapper(
                self.body.genotype.MutateWithConstraints(
                    False,
                    multineat.SearchMode.BLENDED,
                    innov_db,
                    self._MULTINEAT_PARAMS,
                    multineat_rng,
                )
            )
        )

    @classmethod
    def crossover_body(
        cls,
        parent1: Self,
        parent2: Self,
        rng: np.random.Generator,
    ) -> BodyGenotypeV2:
        """
        Perform crossover between two genotypes.

        :param parent1: The first genotype.
        :param parent2: The second genotype.
        :param rng: Random number generator.
        :returns: A newly created genotype.
        """
        multineat_rng = multineat_rng_from_random(rng)

        return BodyGenotypeV2(
            MultineatGenotypePickleWrapper(
                parent1.body.genotype.MateWithConstraints(
                    parent2.body.genotype,
                    False,
                    False,
                    multineat_rng,
                    cls._MULTINEAT_PARAMS,
                )
            )
        )

    def develop_body(self) -> BodyV2:
        """
        Develop the genotype into a modular robot.

        :returns: The created robot.
        """
        return develop(self.body.genotype)
