from __future__ import annotations

import multineat
import numpy as np
import sqlalchemy.orm as orm
from sqlalchemy import event
from sqlalchemy.engine import Connection
from typing_extensions import Self

from revolve2.modular_robot.body.base import Body

from .._multineat_rng_from_random import multineat_rng_from_random
from .._random_multineat_genotype import random_multineat_genotype
from ._brain_cpg_network_neighbor_v1 import BrainCpgNetworkNeighborV1
from ._multineat_params import get_multineat_params

_MULTINEAT_PARAMS = get_multineat_params()


class BrainGenotypeCpgOrm(orm.MappedAsDataclass, kw_only=True):
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
    ) -> BrainGenotypeCpgOrm:
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

        return BrainGenotypeCpgOrm(brain=brain)

    def mutate_brain(
        self,
        innov_db: multineat.InnovationDatabase,
        rng: np.random.Generator,
    ) -> BrainGenotypeCpgOrm:
        """
        Mutate this genotype.

        This genotype will not be changed; a mutated copy will be returned.

        :param innov_db: Multineat innovation database. See Multineat library.
        :param rng: Random number generator.
        :returns: A mutated copy of the provided genotype.
        """
        multineat_rng = multineat_rng_from_random(rng)

        return BrainGenotypeCpgOrm(
            brain=self.brain.MutateWithConstraints(
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
    ) -> BrainGenotypeCpgOrm:
        """
        Perform crossover between two genotypes.

        :param parent1: The first genotype.
        :param parent2: The second genotype.
        :param rng: Random number generator.
        :returns: A newly created genotype.
        """
        multineat_rng = multineat_rng_from_random(rng)

        return BrainGenotypeCpgOrm(
            brain=parent1.brain.MateWithConstraints(
                parent2.brain,
                False,
                False,
                multineat_rng,
                _MULTINEAT_PARAMS,
            )
        )

    def develop_brain(self, body: Body) -> BrainCpgNetworkNeighborV1:
        """
        Develop the genotype into a modular robot.

        :param body: The body to develop the brain for.
        :returns: The created robot.
        """
        return BrainCpgNetworkNeighborV1(genotype=self.brain, body=body)


@event.listens_for(BrainGenotypeCpgOrm, "before_update", propagate=True)
@event.listens_for(BrainGenotypeCpgOrm, "before_insert", propagate=True)
def _serialize_brain(
    mapper: orm.Mapper[BrainGenotypeCpgOrm],
    connection: Connection,
    target: BrainGenotypeCpgOrm,
) -> None:
    target._serialized_brain = target.brain.Serialize()


@event.listens_for(BrainGenotypeCpgOrm, "load", propagate=True)
def _deserialize_brain(target: BrainGenotypeCpgOrm, context: orm.QueryContext) -> None:
    brain = multineat.Genome()
    brain.Deserialize(target._serialized_brain)
    target.brain = brain
