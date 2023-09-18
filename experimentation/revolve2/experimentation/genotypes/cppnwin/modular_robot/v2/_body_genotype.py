from __future__ import annotations

from dataclasses import dataclass

import multineat
import numpy as np
from revolve2.experimentation.genotypes.cppnwin._multineat_genotype_pickle_wrapper import (
    MultineatGenotypePickleWrapper,
)
from revolve2.experimentation.genotypes.cppnwin._multineat_rng_from_random import (
    multineat_rng_from_random,
)
from revolve2.experimentation.genotypes.cppnwin._random_multineat_genotype import (
    random_multineat_genotype,
)
from revolve2.experimentation.genotypes.cppnwin.modular_robot.v2._body_develop import (
    develop,
)
from revolve2.modular_robot import Body
from typing_extensions import Self


def _make_multineat_params() -> multineat.Parameters:
    multineat_params = multineat.Parameters()

    multineat_params.MutateRemLinkProb = 0.02
    multineat_params.RecurrentProb = 0.0
    multineat_params.OverallMutationRate = 0.15
    multineat_params.MutateAddLinkProb = 0.08
    multineat_params.MutateAddNeuronProb = 0.01
    multineat_params.MutateWeightsProb = 0.90
    multineat_params.MaxWeight = 8.0
    multineat_params.WeightMutationMaxPower = 0.2
    multineat_params.WeightReplacementMaxPower = 1.0
    multineat_params.MutateActivationAProb = 0.0
    multineat_params.ActivationAMutationMaxPower = 0.5
    multineat_params.MinActivationA = 0.05
    multineat_params.MaxActivationA = 6.0

    multineat_params.MutateNeuronActivationTypeProb = 0.03

    multineat_params.MutateOutputActivationFunction = False

    multineat_params.ActivationFunction_SignedSigmoid_Prob = 0.0
    multineat_params.ActivationFunction_UnsignedSigmoid_Prob = 0.0
    multineat_params.ActivationFunction_Tanh_Prob = 1.0
    multineat_params.ActivationFunction_TanhCubic_Prob = 0.0
    multineat_params.ActivationFunction_SignedStep_Prob = 1.0
    multineat_params.ActivationFunction_UnsignedStep_Prob = 0.0
    multineat_params.ActivationFunction_SignedGauss_Prob = 1.0
    multineat_params.ActivationFunction_UnsignedGauss_Prob = 0.0
    multineat_params.ActivationFunction_Abs_Prob = 0.0
    multineat_params.ActivationFunction_SignedSine_Prob = 1.0
    multineat_params.ActivationFunction_UnsignedSine_Prob = 0.0
    multineat_params.ActivationFunction_Linear_Prob = 1.0

    multineat_params.MutateNeuronTraitsProb = 0.0
    multineat_params.MutateLinkTraitsProb = 0.0

    multineat_params.AllowLoops = False

    return multineat_params


_MULTINEAT_PARAMS = _make_multineat_params()


@dataclass
class BodyGenotype:
    """CPPNWIN body genotype."""

    _NUM_INITIAL_MUTATIONS = 5

    body: MultineatGenotypePickleWrapper

    @classmethod
    def random_body(
        cls,
        innov_db: multineat.InnovationDatabase,
        rng: np.random.Generator,
    ) -> BodyGenotype:
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
                multineat_params=_MULTINEAT_PARAMS,
                output_activation_func=multineat.ActivationFunction.TANH,
                num_inputs=5,  # bias(always 1), pos_x, pos_y, pos_z, chain_length
                num_outputs=6,  # empty, brick, activehinge, rot0, rot90, attachment_position
                num_initial_mutations=cls._NUM_INITIAL_MUTATIONS,
            )
        )

        return BodyGenotype(body)

    def mutate_body(
        self,
        innov_db: multineat.InnovationDatabase,
        rng: np.random.Generator,
    ) -> BodyGenotype:
        """
        Mutate this genotype.

        This genotype will not be changed; a mutated copy will be returned.

        :param innov_db: Multineat innovation database. See Multineat library.
        :param rng: Random number generator.
        :returns: A mutated copy of the provided genotype.
        """
        multineat_rng = multineat_rng_from_random(rng)

        return BodyGenotype(
            MultineatGenotypePickleWrapper(
                self.body.genotype.MutateWithConstraints(
                    False,
                    multineat.SearchMode.BLENDED,
                    innov_db,
                    _MULTINEAT_PARAMS,
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
    ) -> BodyGenotype:
        """
        Perform crossover between two genotypes.

        :param parent1: The first genotype.
        :param parent2: The second genotype.
        :param rng: Random number generator.
        :returns: A newly created genotype.
        """
        multineat_rng = multineat_rng_from_random(rng)

        return BodyGenotype(
            MultineatGenotypePickleWrapper(
                parent1.body.genotype.MateWithConstraints(
                    parent2.body.genotype,
                    False,
                    False,
                    multineat_rng,
                    _MULTINEAT_PARAMS,
                )
            )
        )

    def develop_body(self) -> Body:
        """
        Develop the genotype into a modular robot.

        :returns: The created robot.
        """
        return develop(self.body.genotype)
