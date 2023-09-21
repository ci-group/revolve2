from revolve2.experimentation.genotypes.cppnwin.modular_robot._body_genotype import BodyGenotype
import multineat
from revolve2.experimentation.genotypes.cppnwin._multineat_genotype_pickle_wrapper import (
    MultineatGenotypePickleWrapper,
)
from revolve2.experimentation.genotypes.cppnwin._multineat_rng_from_random import (
    multineat_rng_from_random,
)
from revolve2.experimentation.genotypes.cppnwin._random_multineat_genotype import (
    random_multineat_genotype,
)

from ._body_develop import develop
import numpy as np
from revolve2.modular_robot import Body, PropertySet

class BodyGenotypeV2(BodyGenotype):
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
                multineat_params=cls._MULTINEAT_PARAMS,
                output_activation_func=multineat.ActivationFunction.TANH,
                num_inputs=5,  # bias(always 1), pos_x, pos_y, pos_z, chain_length
                num_outputs=6,  # empty, brick, activehinge, rot0, rot90, attachment_position
                num_initial_mutations=cls._NUM_INITIAL_MUTATIONS,
            )
        )

        return BodyGenotype(body)

    def develop_body(self, property_set: PropertySet) -> Body:
        """
        Develop the genotype into a modular robot.

        :param property_set: The property set of the body.
        :returns: The created robot.
        """
        return develop(self.body.genotype, property_set)

