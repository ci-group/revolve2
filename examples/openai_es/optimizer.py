import math
from random import Random
from typing import List, Tuple

import numpy as np
import numpy.typing as npt
from pyrr import Quaternion, Vector3
from sqlalchemy.ext.asyncio import AsyncEngine
from sqlalchemy.ext.asyncio.session import AsyncSession

from revolve2.actor_controller import ActorController
from revolve2.core.modular_robot import ActiveHinge, Body, ModularRobot
from revolve2.core.modular_robot.brains import Cpg
from revolve2.core.optimization import ProcessIdGen
from revolve2.core.optimization.ea.openai_es import OpenaiESOptimizer
from revolve2.core.physics.running import (
    ActorControl,
    ActorState,
    Batch,
    Environment,
    PosedActor,
    Runner,
)
from revolve2.runners.isaacgym import LocalRunner


class Optimizer(OpenaiESOptimizer):
    _body: Body
    _num_internal_weights: int

    _runner: Runner
    _controllers: List[ActorController]

    _simulation_time: int
    _sampling_frequency: float
    _control_frequency: float

    _num_generations: int

    async def ainit_new(  # type: ignore # TODO for now ignoring mypy complaint about LSP problem, override parent's ainit
        self,
        database: AsyncEngine,
        session: AsyncSession,
        process_id: int,
        process_id_gen: ProcessIdGen,
        rng: Random,
        population_size: int,
        sigma: float,
        learning_rate: float,
        robot_body: Body,
        simulation_time: int,
        sampling_frequency: float,
        control_frequency: float,
        num_generations: int,
    ) -> None:
        self._body = robot_body
        active_hinges = self._body.find_active_hinges()
        connections = Cpg._find_connections(self._body, active_hinges)
        self._num_internal_weights = len(active_hinges)

        nprng = np.random.Generator(
            np.random.PCG64(rng.randint(0, 2**63))
        )  # rng is currently not numpy, but this would be very convenient. do this until that is resolved.
        initial_mean = nprng.standard_normal(len(active_hinges) + len(connections))

        await super().ainit_new(
            database=database,
            session=session,
            process_id=process_id,
            process_id_gen=process_id_gen,
            rng=rng,
            population_size=population_size,
            sigma=sigma,
            learning_rate=learning_rate,
            initial_mean=initial_mean,
        )

        self._init_runner()

        self._simulation_time = simulation_time
        self._sampling_frequency = sampling_frequency
        self._control_frequency = control_frequency
        self._num_generations = num_generations

    async def ainit_from_database(  # type: ignore # see comment at ainit_new
        self,
        database: AsyncEngine,
        session: AsyncSession,
        process_id: int,
        process_id_gen: ProcessIdGen,
        rng: Random,
        robot_body: Body,
        simulation_time: int,
        sampling_frequency: float,
        control_frequency: float,
        num_generations: int,
    ) -> bool:
        if not await super().ainit_from_database(
            database=database,
            session=session,
            process_id=process_id,
            process_id_gen=process_id_gen,
            rng=rng,
        ):
            return False

        self._body = robot_body
        active_hinges = self._body.find_active_hinges()
        self._num_internal_weights = len(active_hinges)

        self._init_runner()

        self._simulation_time = simulation_time
        self._sampling_frequency = sampling_frequency
        self._control_frequency = control_frequency
        self._num_generations = num_generations

        return True

    def _init_runner(self) -> None:
        self._runner = LocalRunner(LocalRunner.SimParams(), headless=True)

    async def _evaluate_population(
        self,
        database: AsyncEngine,
        process_id: int,
        process_id_gen: ProcessIdGen,
        population: npt.NDArray[np.float_],
    ) -> npt.NDArray[np.float_]:
        batch = Batch(
            simulation_time=self._simulation_time,
            sampling_frequency=self._sampling_frequency,
            control_frequency=self._control_frequency,
            control=self._control,
        )

        self._controllers = []

        for weights in population:
            brain = Brain(
                weights.tolist()[: self._num_internal_weights],
                weights.tolist()[self._num_internal_weights :],
            )
            actor, controller = ModularRobot(
                self._body, brain
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
                )
            )
            batch.environments.append(env)

        states = await self._runner.run_batch(batch)

        return np.array(
            [
                self._calculate_fitness(
                    states[0][1].envs[i].actor_states[0],
                    states[-1][1].envs[i].actor_states[0],
                )
                for i in range(len(population))
            ]
        )

    def _control(self, dt: float, control: ActorControl) -> None:
        for control_i, controller in enumerate(self._controllers):
            controller.step(dt)
            control.set_dof_targets(control_i, 0, controller.get_dof_targets())

    @staticmethod
    def _calculate_fitness(begin_state: ActorState, end_state: ActorState) -> float:
        # TODO simulation can continue slightly passed the defined sim time.

        # distance traveled on the xy plane
        return math.sqrt(
            (begin_state.position[0] - end_state.position[0]) ** 2
            + ((begin_state.position[1] - end_state.position[1]) ** 2)
        )

    def _must_do_next_gen(self) -> bool:
        return self.generation_number != self._num_generations


class Brain(Cpg):
    _internal_weights: List[float]
    _external_weights: List[float]

    def __init__(
        self, internal_weights: List[float], external_weights: List[float]
    ) -> None:
        self._internal_weights = internal_weights
        self._external_weights = external_weights

    def _make_weights(
        self,
        active_hinges: List[ActiveHinge],
        connections: List[Tuple[ActiveHinge, ActiveHinge]],
        body: Body,
    ) -> Tuple[List[float], List[float]]:
        return (self._internal_weights, self._external_weights)
