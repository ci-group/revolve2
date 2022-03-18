from __future__ import annotations

import math
from random import Random
from typing import List, Tuple

import multineat
from genotype import Genotype, mutate, crossover, develop
from fitness import Fitness
from pyrr import Quaternion, Vector3

import revolve2.core.optimization.ea.population_management as population_management
import revolve2.core.optimization.ea.selection as selection
from revolve2.actor_controller import ActorController
from revolve2.core.optimization.ea import EvolutionaryOptimizer
from revolve2.core.physics.running import (
    ActorControl,
    ActorState,
    Batch,
    Environment,
    PosedActor,
    Runner,
    State,
)
from revolve2.runners.isaacgym import LocalRunner
from revolve2.core.database import Database
from revolve2.core.optimization import ProcessIdGen


class Optimizer(EvolutionaryOptimizer["Optimizer", Genotype, Fitness]):
    _runner: Runner

    _controllers: List[ActorController]

    _innov_db_body: multineat.InnovationDatabase
    _innov_db_brain: multineat.InnovationDatabase

    _rng: Random

    _simulation_time: int
    _sampling_frequency: float
    _control_frequency: float

    _num_generations: int

    async def ainit_new(
        self,
        database: Database,
        process_id: int,
        initial_population: List[Genotype],
        rng: Random,
        process_id_gen: ProcessIdGen,
        innov_db_body: multineat.InnovationDatabase,
        innov_db_brain: multineat.InnovationDatabase,
        simulation_time: int,
        sampling_frequency: float,
        control_frequency: float,
        num_generations: int,
        offspring_size: int,
    ) -> None:
        await super().ainit_new(
            database=database,
            process_id=process_id,
            process_id_gen=process_id_gen,
            genotype_type=Genotype,
            fitness_type=Fitness,
            offspring_size=offspring_size,
            initial_population=initial_population,
        )

        self._runner = LocalRunner(LocalRunner.SimParams(), headless=True)
        self._innov_db_body = innov_db_body
        self._innov_db_brain = innov_db_brain
        self._rng = rng
        self._simulation_time = simulation_time
        self._sampling_frequency = sampling_frequency
        self._control_frequency = control_frequency
        self._num_generations = num_generations

        # TODO database things

    async def ainit_from_database(
        self,
        database: Database,
        process_id: int,
        rng: Random,
        process_id_gen: ProcessIdGen,
    ) -> bool:
        if not await super().ainit_from_database(
            database=database,
            process_id=process_id,
            process_id_gen=process_id_gen,
            genotype_type=Genotype,
            fitness_type=Fitness,
        ):
            return False
        self._rng = rng
        return True  # TODO

    def _select_parents(
        self,
        population: List[Genotype],
        fitnesses: List[Fitness],
        num_parent_groups: int,
    ) -> List[List[int]]:
        return [
            selection.multiple_unique(
                population,
                fitnesses,
                2,
                lambda _, fitnesses: selection.tournament(self._rng, fitnesses, k=2),
            )
            for _ in range(num_parent_groups)
        ]

    def _select_survivors(
        self,
        old_individuals: List[Genotype],
        old_fitnesses: List[Fitness],
        new_individuals: List[Genotype],
        new_fitnesses: List[Fitness],
        num_survivors: int,
    ) -> Tuple[List[int], List[int]]:
        assert len(old_individuals) == num_survivors

        return population_management.steady_state(
            old_individuals,
            old_fitnesses,
            new_individuals,
            new_fitnesses,
            lambda _, fitnesses: selection.tournament(self._rng, fitnesses, k=2),
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
        database: Database,
        process_id: int,
        process_id_gen: ProcessIdGen,
    ) -> List[Fitness]:
        batch = Batch(
            simulation_time=self._simulation_time,
            sampling_frequency=self._sampling_frequency,
            control_frequency=self._control_frequency,
            control=self._control,
        )

        self._controllers = []

        for genotype in genotypes:
            actor, controller = develop(genotype).make_actor_and_controller()
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
        # self._save_states(states, database, dbview) TODO

        return [
            self._calculate_fitness(
                states[0][1].envs[i].actor_states[0],
                states[-1][1].envs[i].actor_states[0],
            )
            for i in range(len(genotypes))
        ]

    def _control(self, dt: float, control: ActorControl) -> None:
        for control_i, controller in enumerate(self._controllers):
            controller.step(dt)
            control.set_dof_targets(control_i, 0, controller.get_dof_targets())

    """
    def _save_states(
        self, states: List[Tuple[float, State]], database: Database, db_node: Node
    ) -> None:
        with database.begin_transaction() as txn:
            db_node.set_db_data(
                txn,
                [
                    {"time": time, "actors": actors.serialize()}
                    for (time, actors) in states
                ],
            )
    """

    @staticmethod
    def _calculate_fitness(begin_state: ActorState, end_state: ActorState) -> Fitness:
        # TODO simulation can continue slightly passed the defined sim time.

        # distance traveled on the xy plane
        return Fitness(
            math.sqrt(
                (begin_state.position[0] - end_state.position[0]) ** 2
                + ((begin_state.position[1] - end_state.position[1]) ** 2)
            )
        )
