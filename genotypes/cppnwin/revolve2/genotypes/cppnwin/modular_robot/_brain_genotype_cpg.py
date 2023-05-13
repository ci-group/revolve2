from __future__ import annotations

import multineat
import numpy as np
import sqlalchemy.orm as orm
from sqlalchemy import event
from sqlalchemy.engine import Connection
from typing_extensions import Self

from .._multineat_rng_from_random import multineat_rng_from_random
from .._random_multineat_genotype import random_multineat_genotype
from ._brain_cpg_network_neighbor_v1 import BrainCpgNetworkNeighborV1


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


class BrainGenotypeCpg(orm.MappedAsDataclass):
    """An SQLAlchemy model for a CPPNWIN cpg brain genotype."""

    _NUM_INITIAL_MUTATIONS = 5

    brain: multineat.Genome

    _serialized_brain: orm.Mapped[str] = orm.mapped_column(
        "serialized_brain", init=False, nullable=False
    )

    @classmethod
    def random_brain(
        cls,
        innov_db: multineat.InnovationDatabase,
        rng: np.random.Generator,
    ) -> BrainGenotypeCpg:
        """
        Create a random genotype.

        :param innov_db: Multineat innovation database. See Multineat library.
        :param rng: Random number generator.
        :returns: The created genotype.
        """
        multineat_rng = multineat_rng_from_random(rng)

        brain = random_multineat_genotype(
            innov_db=innov_db,
            rng=multineat_rng,
            multineat_params=_MULTINEAT_PARAMS,
            output_activation_func=multineat.ActivationFunction.SIGNED_SINE,
            num_inputs=7,  # bias(always 1), x1, y1, z1, x2, y2, z2
            num_outputs=1,  # weight
            num_initial_mutations=cls._NUM_INITIAL_MUTATIONS,
        )

        return BrainGenotypeCpg(brain)

    def mutate_brain(
        self,
        innov_db: multineat.InnovationDatabase,
        rng: np.random.Generator,
    ) -> BrainGenotypeCpg:
        """
        Mutate this genotype.

        This genotype will not be changed; a mutated copy will be returned.

        :param innov_db: Multineat innovation database. See Multineat library.
        :param rng: Random number generator.
        :returns: A mutated copy of the provided genotype.
        """
        multineat_rng = multineat_rng_from_random(rng)

        return BrainGenotypeCpg(
            self.brain.MutateWithConstraints(
                False,
                multineat.SearchMode.BLENDED,
                innov_db,
                _MULTINEAT_PARAMS,
                multineat_rng,
            )
        )

    @classmethod
    def crossover_brain(
        cls,
        parent1: Self,
        parent2: Self,
        rng: np.random.Generator,
    ) -> BrainGenotypeCpg:
        """
        Perform crossover between two genotypes.

        :param parent1: The first genotype.
        :param parent2: The second genotype.
        :param rng: Random number generator.
        :returns: A newly created genotype.
        """
        multineat_rng = multineat_rng_from_random(rng)

        return BrainGenotypeCpg(
            parent1.brain.MateWithConstraints(
                parent2.brain,
                False,
                False,
                multineat_rng,
                _MULTINEAT_PARAMS,
            )
        )

    def develop_brain(self) -> BrainCpgNetworkNeighborV1:
        """
        Develop the genotype into a modular robot.

        :returns: The created robot.
        """
        return BrainCpgNetworkNeighborV1(self.brain)


@event.listens_for(BrainGenotypeCpg, "before_update", propagate=True)
@event.listens_for(BrainGenotypeCpg, "before_insert", propagate=True)
def _serialize_brain(
    mapper: orm.Mapper[BrainGenotypeCpg],
    connection: Connection,
    target: BrainGenotypeCpg,
) -> None:
    target._serialized_brain = target.brain.Serialize()


@event.listens_for(BrainGenotypeCpg, "load", propagate=True)
def _deserialize_brain(target: BrainGenotypeCpg, context: orm.QueryContext) -> None:
    brain = multineat.Genome()
    brain.Deserialize(target._serialized_brain)
    target.brain = brain
