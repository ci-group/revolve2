from __future__ import annotations

import math
from random import Random
from typing import List, Optional, Tuple

import multineat
from genotype import Genotype
from pyrr import Quaternion, Vector3

import revolve2.core.optimization.ea.population_management as population_management
import revolve2.core.optimization.ea.selection as selection
from revolve2.actor_controller import ActorController
from revolve2.core.database import Database, Node
from revolve2.core.optimization.ea import EvolutionaryOptimizer, Individual
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
        self._runner = LocalRunner(LocalRunner.SimParams(), headless=True)
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
        assert len(parents) == 2
        return Genotype.crossover(parents[0], parents[1], self._rng)

    def _mutate(self, individual: Genotype) -> Genotype:
        return individual.mutate(self._innov_db_body, self._innov_db_brain, self._rng)

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
        with database.begin_transaction() as txn:
            db_node.set_db_data(
                txn,
                [
                    {"time": time, "actors": actors.serialize()}
                    for (time, actors) in states
                ],
            )

    @staticmethod
    def _calculate_fitness(begin_state: ActorState, end_state: ActorState) -> float:
        # TODO simulation can continue slightly passed the defined sim time.

        # distance traveled on the xy plane
        return math.sqrt(
            (begin_state.position[0] - end_state.position[0]) ** 2
            + ((begin_state.position[1] - end_state.position[1]) ** 2)
        )
