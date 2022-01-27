from __future__ import annotations

import sys
from random import Random

import multineat
from revolve2.core.database import StaticData
from revolve2.core.database.serialization import Serializable, SerializeError
from revolve2.core.optimization.ea.modular_robot import BodybrainGenotype
from revolve2.genotypes.cppnwin import BodyGenotypeV1, BrainGenotypeCpgV1


class Genotype(BodybrainGenotype[BodyGenotypeV1, BrainGenotypeCpgV1], Serializable):
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

    @classmethod
    def random(
        cls,
        innov_db_body: multineat.InnovationDatabase,
        innov_db_brain: multineat.InnovationDatabase,
        rng: Random,
        num_initial_mutations: int,
    ) -> Genotype:
        multineat_rng = cls._multineat_rng_from_random(rng)

        body = BodyGenotypeV1.random(
            innov_db_body,
            multineat_rng,
            cls._MULTINEAT_PARAMS,
            multineat.ActivationFunction.TANH,
            num_initial_mutations,
        )

        brain = BrainGenotypeCpgV1.random(
            innov_db_brain,
            multineat_rng,
            cls._MULTINEAT_PARAMS,
            multineat.ActivationFunction.SIGNED_SINE,
            num_initial_mutations,
        )

        return cls(body, brain)

    def mutate(
        self,
        innov_db_body: multineat.InnovationDatabase,
        innov_db_brain: multineat.InnovationDatabase,
        rng: Random,
    ) -> Genotype:
        multineat_rng = self._multineat_rng_from_random(rng)

        return Genotype(
            self._body_genotype.mutate(
                self._MULTINEAT_PARAMS, innov_db_body, multineat_rng
            ),
            self._brain_genotype.mutate(
                self._MULTINEAT_PARAMS, innov_db_brain, multineat_rng
            ),
        )

    @classmethod
    def crossover(
        cls,
        parent1: Genotype,
        parent2: Genotype,
        rng: Random,
    ):
        multineat_rng = cls._multineat_rng_from_random(rng)

        return Genotype(
            BodyGenotypeV1.crossover(
                parent1._body_genotype,
                parent2._body_genotype,
                Genotype._MULTINEAT_PARAMS,
                multineat_rng,
                False,
                False,
            ),
            BrainGenotypeCpgV1.crossover(
                parent1._brain_genotype,
                parent2._brain_genotype,
                Genotype._MULTINEAT_PARAMS,
                multineat_rng,
                False,
                False,
            ),
        )

    @staticmethod
    def _multineat_rng_from_random(rng: Random) -> multineat.RNG:
        multineat_rng = multineat.RNG()
        multineat_rng.Seed(rng.randint(0, sys.maxsize))
        return multineat_rng

    def serialize(self) -> StaticData:
        test = {
            "body": self._body_genotype.serialize(),
            "brain": self._brain_genotype.serialize(),
        }
        return test

    @classmethod
    def deserialize(cls, data: StaticData) -> Genotype:
        if type(data) != dict:
            raise SerializeError()
        return cls(
            BodyGenotypeV1.deserialize(data["body"]),
            BrainGenotypeCpgV1.deserialize(data["brain"]),
        )
