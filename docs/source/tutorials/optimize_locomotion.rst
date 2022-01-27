=======================================================================
Optimizing a modular robot for locomotion using CPPNWIN and Isaac Gym
=======================================================================
The final results of this tutorial are available at ``<revolve2_source>/examples/optimize_modular>``.

-------------------
What you will learn
-------------------

* How to use the ``CPPNWIN`` genotype supplementary library.
* How to combine modular robots with evolutionary optimization.
* How to manually insert into a database.

-------------
Prerequisites
-------------

* Tutorial :ref:`tutorials/simple_optimization:A simple optimization process with a database`
* Tutorial :ref:`tutorials/simulate_robot_isaac:Creating a modular robot and simulating it in the Isaac Gym environment`
* Have the supplementary library ``genotype CPPNWIN`` :ref:`installed <installation/genotypes/cppnwin:CPPNWIN genotype supplemental library>`.
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
``BodyGenotypeV1`` and ``BrainGenotypeCpgV1`` are body and brain CPPNWIN genotypes respectively, provided by the ``CPPNWIN`` supplementary library.
They use the CPPNWIN network to create a modular robot body and weights for a corresponding CPG brain::

    from __future__ import annotations
    from random import Random
    from revolve2.core.database import StaticData
    from revolve2.core.database.serialization import Serializable
    from revolve2.core.optimization.ea.modular_robot import BodybrainGenotype
    from revolve2s.cppnwin import BodyGenotypeV1, BrainGenotypeCpgV1


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
The optimizer will look similar to the optimizer from the previous evolutionary optimization tutorial.
Add the innovation databases as they will be shared between all genotypes.
Additionally, instead of a single controller you will store a controller for each individual robot::

    # optimizer.py

    from __future__ import annotations
    from genotype import Genotype
    from revolve2.core.optimization.ea import EvolutionaryOptimizer, Individual
    from revolve2.core.physics.env import Runner, ActorControl
    import multineat
    from typing import List, Optional
    from revolve2.core.database import Database, Node
    from random import Random
    from revolve2.envs.isaacgym import LocalRunner
    from revolve2.core.physics.control import ActorController
    import revolve2.core.optimization.ea.selection as selection
    import revolve2.core.optimization.ea.population_management as population_management

    class Optimizer(EvolutionaryOptimizer[Genotype, float]):
        _runner: Runner

        _controllers: List[ActorController]

        _innov_db_body: multineat.InnovationDatabase
        _innov_db_brain: multineat.InnovationDatabase

        _simulation_time: int
        _sampling_frequency: float
        _control_frequency: float

        _num_generations: int

        def __init__(self) -> None:
            pass

        async def create(
            database: Database,
            initial_population: List[Genotype],
            initial_fitness: Optional[List[float]],
            rng: Random,
            innov_db_body: multineat.InnovationDatabase,
            innov_db_brain: multineat.InnovationDatabase,
            simulation_time: int,
            sampling_frequency: float,
            control_frequency: float,
            num_generations: int,
            population_size: int,
            offspring_size: int,
        ) -> Optimizer:
            self = Optimizer()

            await super(Optimizer, self).asyncinit(
                database,
                database.root,
                rng,
                population_size,
                offspring_size,
                initial_population,
                initial_fitness,
            )
            self._runner = LocalRunner(LocalRunner.SimParams())
            self._innov_db_body = innov_db_body
            self._innov_db_brain = innov_db_brain
            self._simulation_time = simulation_time
            self._sampling_frequency = sampling_frequency
            self._control_frequency = control_frequency
            self._num_generations = num_generations

            return self

        def _select_parents(
            self,
            generation: List[Individual[Genotype, float]],
            num_parents: int,
        ) -> List[List[Individual[Genotype, float]]]:
            return [
                [
                    i[0]
                    for i in selection.multiple_unique(
                        [(i, i.fitness) for i in generation],
                        2,
                        lambda gen: selection.tournament(self._rng, gen, k=2),
                    )
                ]
                for _ in range(num_parents)
            ]

        def _select_survivors(
            self,
            old_individuals: List[Individual[Genotype, float]],
            new_individuals: List[Individual[Genotype, float]],
            num_survivors: int,
        ) -> List[Individual[Genotype, float]]:
            assert len(old_individuals) == num_survivors

            return [
                i[0]
                for i in population_management.steady_state(
                    [(i, i.fitness) for i in old_individuals],
                    [(i, i.fitness) for i in new_individuals],
                    lambda pop: selection.tournament(self._rng, pop, k=2),
                )
            ]

        def _must_do_next_gen(self) -> bool:
            return self.generation_index != self._num_generations

        def _crossover(self, parents: List[Genotype]) -> Genotype:
            raise NotImplementedError()

        def _mutate(self, individual: Genotype) -> Genotype:
            raise NotImplementedError()

        async def _evaluate_generation(
            self, individuals: List[Genotype], database: Database, dbview: Node
        ) -> List[float]:
            raise NotImplementedError()

        def _control(self, dt: float, control: ActorControl) -> None:
            raise NotImplementedError()

``crossover`` and ``mutate`` can use the functions defined in your ``Genotype``::

    def _crossover(self, parents: List[Genotype]) -> Genotype:
        assert len(parents) == 2
        return Genotype.crossover(parents[0], parents[1], self._rng)

    def _mutate(self, individual: Genotype) -> Genotype:
        return individual.mutate(self._innov_db_body, self._innov_db_brain, self._rng)

Evaluation works similar to the simulator created in the previous Isaac Gym environment tutorial, with two small differences.
Firstly, there are multiple robots. Store the controllers in their array and call all of them in the control function.
Secondly, the robots have unpredicatable bodies, so use the modular robot's built-in axis aligned bounding box(aabb) function to find out how high off the ground you need to position it.
Thirdly, save the simulation states in the database. This gives you a history so you can analyze each robot later.
The amound of simulation states depends on the ``sampling_frequency`` defined earlier.
And lastly, calculate and return the fitness of the robot, based on the simulation history.::


    from revolve2.core.physics.env import Batch, Environment, PosedActor, State, ActorState
    from pyrr import Vector3, Quaternion
    from typing import Tuple
    import math

    # ...

    async def _evaluate_generation(
        self, individuals: List[Genotype], database: Database, dbview: Node
    ) -> List[float]:
        batch = Batch(
            simulation_time=self._simulation_time,
            sampling_frequency=self._sampling_frequency,
            control_frequency=self._control_frequency,
            control=self._control,
        )

        self._controllers = []

        for individual in individuals:
            actor, controller = individual.develop().make_actor_and_controller()
            bounding_box = actor.calc_aabb()
            self._controllers.append(controller)
            env = Environment()
            env.actors.append(
                PosedActor(
                    actor,
                    Vector3(
                        [
                            0.0,
                            0.0,
                            bounding_box.size.z / 2.0 - bounding_box.offset.z,
                        ]
                    ),
                    Quaternion(),
                )
            )
            batch.environments.append(env)

        states = await self._runner.run_batch(batch)
        self._save_states(states, database, dbview)

        return [
            self._calculate_fitness(
                states[0][1].envs[i].actor_states[0],
                states[-1][1].envs[i].actor_states[0],
            )
            for i in range(len(individuals))
        ]

    def _control(self, dt: float, control: ActorControl) -> None:
        for control_i, controller in enumerate(self._controllers):
            controller.step(dt)
            control.set_dof_targets(control_i, 0, controller.get_dof_targets())

    def _save_states(
        self, states: List[Tuple[float, State]], database: Database, db_node: Node
    ) -> None:
        raise NotImplementedError()

    @staticmethod
    def _calculate_fitness(begin_state: ActorState, end_state: ActorState) -> float:
        raise NotImplementedError()

Implement saving the states to the database.
Any interaction with a database requires a transactions.
Whenever an error occurs everything done within the transaction is rolled back.
The ``EvolutionaryOptimizer`` provides you with an uninitialized ``Node`` for you to write your simulation history to.
Be aware that you can only write to a node one time. Once the transaction completes you cannot write again.
Simply convert your states to an ``Object`` and write it to the node.
An ``Object`` is a superset of ``StaticData``. For now you do not have to worry about this::

    def _save_states(
        self, states: List[Tuple[float, State]], database: Database, db_node: Node
    ) -> None:
        with database.begin_transaction() as txn:
            db_node.set_object(
                txn,
                [
                    {"time": time, "actors": actors.serialize()}
                    for (time, actors) in states
                ],
            )

The last thing you have to do is write the fitness function.
This tutorial simply uses the distance traveled on the xy plane::

    @staticmethod
    def _calculate_fitness(begin_state: ActorState, end_state: ActorState) -> float:
        # distance traveled on the xy plane
        return math.sqrt(
            (begin_state.position[0] - end_state.position[0]) ** 2
            + ((begin_state.position[1] - end_state.position[1]) ** 2)
        )

~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Adding the optimizer to main
~~~~~~~~~~~~~~~~~~~~~~~~~~~~
You can now add the finished optimizer to your ``main`` function.
You will also need to add some extra constants.::

    # optimize.py

    import multineat
    from genotype import Genotype
    from optimizer import Optimizer

    # ...

    async def main():
        # number of initial mutations for body and brain CPPNWIN networks
        NUM_INITIAL_MUTATIONS = 10

        SIMULATION_TIME = 10
        SAMPLING_FREQUENCY = 5
        CONTROL_FREQUENCY = 5

        # ...

        # database
        database = await DbSqlite.create(f"database")

        # multineat innovation databases
        innov_db_body = multineat.InnovationDatabase()
        innov_db_brain = multineat.InnovationDatabase()

        initial_population = [
            Genotype.random(innov_db_body, innov_db_brain, rng, NUM_INITIAL_MUTATIONS)
            for _ in range(POPULATION_SIZE)
        ]

        ep = await Optimizer.create(
            database,
            initial_population=initial_population,
            initial_fitness=None,
            rng=rng,
            innov_db_body=innov_db_body,
            innov_db_brain=innov_db_brain,
            simulation_time=SIMULATION_TIME,
            sampling_frequency=SAMPLING_FREQUENCY,
            control_frequency=CONTROL_FREQUENCY,
            num_generations=NUM_GENERATIONS,
            population_size=POPULATION_SIZE,
            offspring_size=OFFSPRING_SIZE,
        )

        logging.info("Starting optimization process..")

        await ep.run()

        logging.info(f"Finished optimizing.")

    # ...

---------------------
Running and analyzing
---------------------
Run ``optimize.py`` and see your robots evolve.
Depending on your parameters this can take quite a long time.

.. image:: optimize_locomotion_simulator.gif
    :width: 100%

You can analyze the results using the same scripts as from the previous optimization tutorial::

    revolve2.analysis.core.plot_ea_fitness database

.. image:: optimize_locomotion_analysis.png
    :width: 100%

Of course this is a low number of generations, populations size, offspring size, and simulation time, but compare your results to see if the simulation runs correctly.

------------
Contributing
------------
This marks the end of this tutorial. Feedback and contributions are welcome at Revolve2's code repository.