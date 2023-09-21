from __future__ import annotations

import multineat
import numpy as np
import sqlalchemy.orm as orm
from revolve2.experimentation.genotypes.cppnwin._multineat_rng_from_random import (
    multineat_rng_from_random,
)
from revolve2.experimentation.genotypes.cppnwin._random_multineat_genotype import (
    random_multineat_genotype,
)
from revolve2.experimentation.genotypes.cppnwin.modular_robot.v1._body_develop import (
    develop,
)
from revolve2.experimentation.genotypes.cppnwin.modular_robot._body_genotype_orm import BodyGenotypeOrm

from revolve2.modular_robot import Body, PropertySet
from sqlalchemy import event
from sqlalchemy.engine import Connection
from ._body_develop import develop



class BodyGenotypeOrmV1(BodyGenotypeOrm, orm.MappedAsDataclass, kw_only=True):
    """SQLAlchemy model for a CPPNWIN body genotype."""

    _serialized_body: orm.Mapped[str] = orm.mapped_column(
        "serialized_body", init=False, nullable=False
    )

    @classmethod
    def random_body(
        cls,
        innov_db: multineat.InnovationDatabase,
        rng: np.random.Generator,
    ) -> BodyGenotypeOrmV1:
        """
        Create a random genotype.

        :param innov_db: Multineat innovation database. See Multineat library.
        :param rng: Random number generator.
        :returns: The created genotype.
        """
        multineat_rng = multineat_rng_from_random(rng)

        body = random_multineat_genotype(
            innov_db=innov_db,
            rng=multineat_rng,
            multineat_params=cls._MULTINEAT_PARAMS,
            output_activation_func=multineat.ActivationFunction.TANH,
            num_inputs=5,  # bias(always 1), pos_x, pos_y, pos_z, chain_length
            num_outputs=6,  # empty, brick, activehinge, rot0, rot90, attachment_position
            num_initial_mutations=cls._NUM_INITIAL_MUTATIONS,
        )

        return BodyGenotypeOrmV1(body=body)


    def develop_body(self, property_set: PropertySet) -> Body:
        """
        Develop the genotype into a modular robot.

        :param property_set: The propertyset of the body.
        :returns: The created robot.
        """
        return develop(self.body, property_set)


@event.listens_for(BodyGenotypeOrmV1, "before_update", propagate=True)
@event.listens_for(BodyGenotypeOrmV1, "before_insert", propagate=True)
def _update_serialized_body(
    mapper: orm.Mapper[BodyGenotypeOrmV1], connection: Connection, target: BodyGenotypeOrmV1
) -> None:
    target._serialized_body = target.body.Serialize()


@event.listens_for(BodyGenotypeOrmV1, "load", propagate=True)
def _deserialize_body(target: BodyGenotypeOrmV1, context: orm.QueryContext) -> None:
    body = multineat.Genome()
    body.Deserialize(target._serialized_body)
    target.body = body
