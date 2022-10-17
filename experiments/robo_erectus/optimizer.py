"""Optimizer for finding a good modular robot body and brain using CPPNWIN genotypes and simulation using mujoco."""

import math
import logging
import pickle
import wandb
from random import Random
from typing import List, Tuple, Optional

import multineat
import revolve2.core.optimization.ea.generic_ea.population_management as population_management
import revolve2.core.optimization.ea.generic_ea.selection as selection
import sqlalchemy
from measures import *
from genotype import Genotype, GenotypeSerializer, crossover, develop, mutate
from pyrr import Quaternion, Vector3
from revolve2.actor_controller import ActorController
from revolve2.core.database import IncompatibleError
from revolve2.core.database.serializers import FloatSerializer
from revolve2.core.optimization import ProcessIdGen
from revolve2.core.optimization.ea.generic_ea import EAOptimizer
from revolve2.core.physics.running import (
    ActorControl,
    ActorState,
    Batch,
    Environment,
    PosedActor,
    Runner,
)
from revolve2.core.physics.running._results import EnvironmentResults
from revolve2.runners.mujoco import LocalRunner
from sensor_feedback_lib._local_runner import LocalRunner as Feedback_LocalRunner
from sqlalchemy.ext.asyncio import AsyncEngine
from sqlalchemy.ext.asyncio.session import AsyncSession
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.future import select
from fitness import fitness_functions


class Optimizer(EAOptimizer[Genotype, float]):
    """
    Optimizer for the problem.

    Uses the generic EA optimizer as a base.
    """

    _process_id: int

    _runner: Runner

    _controllers: List[ActorController]

    _innov_db_body: multineat.InnovationDatabase
    _innov_db_brain: multineat.InnovationDatabase

    _rng: Random

    _simulation_time: int
    _sampling_frequency: float
    _control_frequency: float

    _num_generations: int

    _fitness_function: str
    _use_feedback: bool

    async def ainit_new(  # type: ignore # TODO for now ignoring mypy complaint about LSP problem, override parent's ainit
        self,
        database: AsyncEngine,
        session: AsyncSession,
        process_id: int,
        process_id_gen: ProcessIdGen,
        initial_population: List[Genotype],
        rng: Random,
        innov_db_body: multineat.InnovationDatabase,
        innov_db_brain: multineat.InnovationDatabase,
        simulation_time: int,
        sampling_frequency: float,
        control_frequency: float,
        num_generations: int,
        offspring_size: int,
        fitness_function: str,
        use_feedback: bool,
    ) -> None:
        """
        Initialize this class async.

        Called when creating an instance using `new`.

        :param database: Database to use for this optimizer.
        :param session: Session to use when saving data to the database during initialization.
        :param process_id: Unique identifier in the completely program specifically made for this optimizer.
        :param process_id_gen: Can be used to create more unique identifiers.
        :param initial_population: List of genotypes forming generation 0.
        :param rng: Random number generator.
        :param innov_db_body: Innovation database for the body genotypes.
        :param innov_db_brain: Innovation database for the brain genotypes.
        :param simulation_time: Time in second to simulate the robots for.
        :param sampling_frequency: Sampling frequency for the simulation. See `Batch` class from physics running.
        :param control_frequency: Control frequency for the simulation. See `Batch` class from physics running.
        :param num_generations: Number of generation to run the optimizer for.
        :param offspring_size: Number of offspring made by the population each generation.
        """
        await super().ainit_new(
            database=database,
            session=session,
            process_id=process_id,
            process_id_gen=process_id_gen,
            genotype_type=Genotype,
            genotype_serializer=GenotypeSerializer,
            fitness_type=float,
            fitness_serializer=FloatSerializer,
            offspring_size=offspring_size,
            initial_population=initial_population,
        )

        self._process_id = process_id
        self._innov_db_body = innov_db_body
        self._innov_db_brain = innov_db_brain
        self._rng = rng
        self._simulation_time = simulation_time
        self._sampling_frequency = sampling_frequency
        self._control_frequency = control_frequency
        self._num_generations = num_generations
        self._fitness_function = fitness_function
        self._use_feedback = use_feedback
        self._init_runner()

        # create database structure if not exists
        # TODO this works but there is probably a better way
        await (await session.connection()).run_sync(DbBase.metadata.create_all)

        # save to database
        self._on_generation_checkpoint(session)

    async def ainit_from_database(  # type: ignore # see comment at ainit_new
        self,
        database: AsyncEngine,
        session: AsyncSession,
        process_id: int,
        process_id_gen: ProcessIdGen,
        rng: Random,
        innov_db_body: multineat.InnovationDatabase,
        innov_db_brain: multineat.InnovationDatabase,
    ) -> bool:
        """
        Try to initialize this class async from a database.

        Called when creating an instance using `from_database`.

        :param database: Database to use for this optimizer.
        :param session: Session to use when loading and saving data to the database during initialization.
        :param process_id: Unique identifier in the completely program specifically made for this optimizer.
        :param process_id_gen: Can be used to create more unique identifiers.
        :param rng: Random number generator.
        :param innov_db_body: Innovation database for the body genotypes.
        :param innov_db_brain: Innovation database for the brain genotypes.
        :returns: True if this complete object could be deserialized from the database.
        :raises IncompatibleError: In case the database is not compatible with this class.
        """
        if not await super().ainit_from_database(
            database=database,
            session=session,
            process_id=process_id,
            process_id_gen=process_id_gen,
            genotype_type=Genotype,
            genotype_serializer=GenotypeSerializer,
            fitness_type=float,
            fitness_serializer=FloatSerializer,
        ):
            return False

        self._process_id = process_id
        self._init_runner()

        opt_row = (
            (
                await session.execute(
                    select(DbOptimizerState)
                    .filter(DbOptimizerState.process_id == process_id)
                    .order_by(DbOptimizerState.generation_index.desc())
                )
            )
            .scalars()
            .first()
        )

        # if this happens something is wrong with the database
        if opt_row is None:
            raise IncompatibleError

        self._simulation_time = opt_row.simulation_time
        self._sampling_frequency = opt_row.sampling_frequency
        self._control_frequency = opt_row.control_frequency
        self._num_generations = opt_row.num_generations

        self._rng = rng
        self._rng.setstate(pickle.loads(opt_row.rng))

        self._innov_db_body = innov_db_body
        self._innov_db_body.Deserialize(opt_row.innov_db_body)
        self._innov_db_brain = innov_db_brain
        self._innov_db_brain.Deserialize(opt_row.innov_db_brain)

        self._fitness_function = opt_row.fitness_function
        self._use_feedback = opt_row.use_feedback

        return True

    def _init_runner(self, headless=True) -> None:
        if self._use_feedback:
            self._runner = Feedback_LocalRunner(headless=headless)
        else:
            self._runner = LocalRunner(headless=headless)

    def _select_parents(
        self,
        population: List[Genotype],
        fitnesses: List[float],
        num_parent_groups: int,
    ) -> List[List[int]]:
        return [
            selection.multiple_unique(
                2,
                population,
                fitnesses,
                lambda _, fitnesses: selection.tournament(self._rng, fitnesses, k=2),
            )
            for _ in range(num_parent_groups)
        ]

    def _select_survivors(
        self,
        old_individuals: List[Genotype],
        old_fitnesses: List[float],
        new_individuals: List[Genotype],
        new_fitnesses: List[float],
        num_survivors: int,
    ) -> Tuple[List[int], List[int]]:
        assert len(old_individuals) == num_survivors

        return population_management.steady_state(
            old_individuals,
            old_fitnesses,
            new_individuals,
            new_fitnesses,
            lambda n, genotypes, fitnesses: selection.multiple_unique(
                n,
                genotypes,
                fitnesses,
                lambda genotypes, fitnesses: selection.tournament(
                    self._rng, fitnesses, k=2
                ),
            ),
        )

    def _must_do_next_gen(self) -> bool:
        return self.generation_index != self._num_generations

    def _crossover(self, parents: List[Genotype]) -> Genotype:
        assert len(parents) == 2
        return crossover(parents[0], parents[1], self._rng)

    def _mutate(self, genotype: Genotype) -> Genotype:
        return mutate(genotype, self._innov_db_body, self._innov_db_brain, self._rng)

    async def _evaluate_generation(
        self,
        genotypes: List[Genotype],
        database: AsyncEngine,
        process_id: int,
        process_id_gen: ProcessIdGen,
    ) -> List[float]:
        if self._use_feedback:
            batch = Batch(
                simulation_time=self._simulation_time,
                sampling_frequency=self._sampling_frequency,
                control_frequency=self._control_frequency,
                control=self._feedback_control,
            )
        else:
            batch = Batch(
                simulation_time=self._simulation_time,
                sampling_frequency=self._sampling_frequency,
                control_frequency=self._control_frequency,
                control=self._control,
            )

        self._controllers = []

        for genotype in genotypes:
            # here the actual controllers are created:
            actor, controller = develop(
                genotype, self._use_feedback
            ).make_actor_and_controller()
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
                    [0.0 for _ in controller.get_dof_targets()],
                ),
            )
            batch.environments.append(env)

        batch_results = await self._runner.run_batch(batch)

        print(f"fitness_function: {self._fitness_function}")
        fitness = [
            fitness_functions[self._fitness_function](environment_result)
            for environment_result, environment in zip(
                batch_results.environment_results, batch.environments
            )
        ]
        displacement = [
            displacement_measure(r) for r in batch_results.environment_results
        ]

        wandb.log(
            {
                "displacement_max": max(displacement),
                "displacement_avg": sum(displacement) / len(displacement),
                "displacement_min": min(displacement),
                "fitness_max": max(fitness),
                "fitness_avg": sum(fitness) / len(fitness),
                "fitness_min": min(fitness),
                "displacement": wandb.Histogram(displacement),
                "max_height_relative_to_avg_height": wandb.Histogram(
                    [
                        max_height_relative_to_avg_height_measure(r)
                        for r in batch_results.environment_results
                    ]
                ),
                "ground_contact_measure": wandb.Histogram(
                    [
                        ground_contact_measure(r)
                        for r in batch_results.environment_results
                    ]
                ),
            }
        )

        return fitness

    def _control(
        self, environment_index: int, dt: float, control: ActorControl
    ) -> None:
        """controller for batch that influnces models in simulation"""
        controller = self._controllers[environment_index]
        controller.step(dt)
        control.set_dof_targets(0, controller.get_dof_targets())

    def _feedback_control(
        self,
        environment_index: int,
        dt: float,
        control: ActorControl,
        sensor_inputs: List[int],
    ) -> None:
        """controller for batch that influnces models in simulation"""
        controller = self._controllers[environment_index]
        controller.step(dt, sensor_inputs)
        control.set_dof_targets(0, controller.get_dof_targets())

    def _on_generation_checkpoint(self, session: AsyncSession) -> None:
        session.add(
            DbOptimizerState(
                process_id=self._process_id,
                generation_index=self.generation_index,
                rng=pickle.dumps(self._rng.getstate()),
                innov_db_body=self._innov_db_body.Serialize(),
                innov_db_brain=self._innov_db_brain.Serialize(),
                simulation_time=self._simulation_time,
                sampling_frequency=self._sampling_frequency,
                control_frequency=self._control_frequency,
                num_generations=self._num_generations,
                fitness_function=self._fitness_function,
            )
        )


DbBase = declarative_base()


class DbOptimizerState(DbBase):
    """Optimizer state."""

    __tablename__ = "optimizer"

    process_id = sqlalchemy.Column(
        sqlalchemy.Integer,
        nullable=False,
        primary_key=True,
    )
    generation_index = sqlalchemy.Column(
        sqlalchemy.Integer, nullable=False, primary_key=True
    )
    rng = sqlalchemy.Column(sqlalchemy.PickleType, nullable=False)
    innov_db_body = sqlalchemy.Column(sqlalchemy.String, nullable=False)
    innov_db_brain = sqlalchemy.Column(sqlalchemy.String, nullable=False)
    simulation_time = sqlalchemy.Column(sqlalchemy.Integer, nullable=False)
    sampling_frequency = sqlalchemy.Column(sqlalchemy.Float, nullable=False)
    control_frequency = sqlalchemy.Column(sqlalchemy.Float, nullable=False)
    num_generations = sqlalchemy.Column(sqlalchemy.Integer, nullable=False)
    fitness_function = sqlalchemy.Column(sqlalchemy.String, nullable=False)
