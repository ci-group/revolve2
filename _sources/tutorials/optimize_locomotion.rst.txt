=======================================================================
Optimizing a modular robot for locomotion using CPPN+NEAT and Isaac Gym
=======================================================================
The final results of this tutorial are available at ``<revolve2_source>/examples/optimize_modular>``.

-------------------
What you will learn
-------------------

* How to use the ``CPPN+NEAT`` genotype supplementary library.
* How to combine modular robots with evolutionary optimization.

-------------
Prerequisites
-------------

* Tutorial :ref:`tutorials/simple_optimization:A simple optimization process with a database`
* Tutorial :ref:`tutorials/simulate_robot_isaac:Creating a modular robot and simulating it in the Isaac Gym environment`
* Have the supplementary library ``genotype CPPN+NEAT`` :ref:`installed <installation/genotypes/cppnneat:Cppn+Neat genotype supplemental library>`.
* Superficial knowledge of the :ref:`CPPNWIN <concepts/cppnwin:CPPNWIN>` genotype, used by ci-group.

------------
Introduction
------------
You will create a program that optimizes a modular robot body and brain for undirected locomotion.
That is, a robot that moves as far as possible, no matter the direction.

--------------
Programming it
--------------
For this tutorial it will be convenient to seperate your code in multiple files.
Create an empty directory and a file ``optimize.py``. This will be your program's entry point.
The baseline code for this program is similar to the previous tutorials'::

    # optimize.py

    import logging
    from random import Random
    from revolve2.core.database.sqlite import Database as DbSqlite

    async def main():
        POPULATION_SIZE = 10
        OFFSPRING_SIZE = 10
        NUM_GENERATIONS = 3

        logging.basicConfig(
            level=logging.DEBUG,
            format="[%(asctime)s] [%(levelname)s] [%(module)s] %(message)s",
        )

        logging.info(f"Starting optimization")

        # random number generator
        rng = Random()
        rng.seed(100)

        # database
        database = await DbSqlite.create(f"database")

        # here you will add your initial population and optimizer later
        # ...

        logging.info("Starting optimization process..")

        # await ep.run()

        logging.info(f"Finished optimizing.")


    if __name__ == "__main__":
        import asyncio

        asyncio.run(main())

~~~~~~~~~~~~
The genotype
~~~~~~~~~~~~

Start by creating genotype in a new file called ``genotype.py``.
The genotype will inherit from ``BodybrainGenotype``, a base class provided by the modular_robot module.
This class stores seperate body and brain genotypes and its ``develop`` function combines these to create a ``ModularRobot``.
``BodyGenotypeV1`` and ``BrainGenotypeCpgV1`` are body and brain CPPNWIN genotypes respectively, provided by the ``CPPN+NEAT`` supplementary library::

    from __future__ import annotations
    from random import Random
    from revolve2.core.database import StaticData
    from revolve2.core.database.serialization import Serializable
    from revolve2.core.optimization.ea.modular_robot import BodybrainGenotype
    from revolve2.genotypes.cppnneat import BodyGenotypeV1, BrainGenotypeCpgV1


    class Genotype(BodybrainGenotype[BodyGenotypeV1, BrainGenotypeCpgV1], Serializable):
        @classmethod
        def random(
            cls,
            rng: Random,
            num_initial_mutations: int,
        ) -> Genotype:
            raise NotImplementedError()

        def mutate(
            self,
            rng: Random,
        ) -> Genotype:
            raise NotImplementedError()

        @classmethod
        def crossover(
            cls,
            parent1: Genotype,
            parent2: Genotype,
            rng: Random,
        ):
            raise NotImplementedError()

        def serialize(self) -> StaticData:
            raise NotImplementedError()

        @classmethod
        def deserialize(cls, data: StaticData) -> Genotype:
            raise NotImplementedError()

The CPPNWIN genotypes each require an innovation database when initialize, so add these as arguments to your random function.
Additionally you have to provide many parameters. These must be set using the ``multineat.Parameters`` structure.
It is convenient to store these parameters are a constant value in your genotype.
Next, multineat has its own random object and does not take the standard python ``Random``.
This is unfortunate, but there is no way around it.
To make sure that your randomness is reproducible and an be set with a single seed, simple seed a new ``multineat.RNG`` object with a value from your ``Random`` object.
Finally, the ``CPPWIN`` supplementary library creates random genotypes by mutating an empty network a few times.
Add a parameters so the number of mutation can be set::


    import multineat
    import sys

    # ...

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

    @staticmethod
    def _multineat_rng_from_random(rng: Random) -> multineat.RNG:
        multineat_rng = multineat.RNG()
        multineat_rng.Seed(rng.randint(0, sys.maxsize))
        return multineat_rng

Mutation is straightforward. Create a new instance of your ``Genotype`` class and use the ``mutate`` functions on your body and brain genotypes::

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

Crossover is very similar. It takes some extra parameters that are irrelevant for now. Take a look at the documentation if you are interested::

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

Serialization and deserializataion is not as easy as in the previous tutorials, as the types contained in this class are not ``StaticData``.
However, they provide ``serialize`` and ``deserialize`` functions of their own.
Simply use these to create a ``StaticData`` representation of your genotype::

    from revolve2.core.database.serialization import SerializeError

    # ...

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

~~~~~~~~~~~~~
The optimizer
~~~~~~~~~~~~~
TODO