"""Evaluator class."""

import asyncio
import math
from typing import List

from pyrr import Quaternion, Vector3
from revolve2.core.modular_robot import ModularRobot
from revolve2.core.physics import Terrain
from revolve2.core.physics.environment_actor_controller import (
    EnvironmentActorController,
)
from revolve2.core.physics.running import ActorState, Batch
from revolve2.core.physics.running import Environment as PhysicsEnv
from revolve2.core.physics.running import PosedActor, Runner
from revolve2.runners.mujoco import LocalRunner
from revolve2.standard_resources import terrains


class Evaluator:
    """Provides evaluation of robots."""

    _runner: Runner
    _terrain: Terrain
    _simulation_time: int
    _sampling_frequency: int
    _control_frequency: int

    def __init__(
        self,
        headless: bool,
        num_simulators: int,
        simulation_time: int,
        sampling_frequency: int,
        control_frequency: int,
    ) -> None:
        """
        Initialize this object.

        :param headless: `headless` parameter for the physics runner.
        :param num_simulators: `num_simulators` parameter for the physics runner.
        :param simulation_time: `simulation_time` parameter for created batches passed to the physics runner.
        :param sampling_frequency: `sampling_frequency` parameter for created batches passed to the physics runner.
        :param control_frequency: `control_frequency` parameter for created batches passed to the physics runner.
        """
        self._runner = LocalRunner(headless=headless, num_simulators=num_simulators)
        self._terrain = terrains.flat()
        self._simulation_time = simulation_time
        self._sampling_frequency = sampling_frequency
        self._control_frequency = control_frequency

    def evaluate(
        self,
        robots: List[ModularRobot],
    ) -> List[float]:
        """
        Evaluate multiple robots.

        Fitness is the distance traveled on the xy plane.

        :param robots: The robots to simulate.
        :returns: Fitnesses of the robots.
        """
        batch = Batch(
            simulation_time=self._simulation_time,
            sampling_frequency=self._sampling_frequency,
            simulation_timestep=0.001,
            control_frequency=self._control_frequency,
        )

        for robot in robots:
            actor, controller = robot.make_actor_and_controller()
            bounding_box = actor.calc_aabb()
            env = PhysicsEnv(EnvironmentActorController(controller))
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
                )
            )
            env.static_geometries.extend(self._terrain.static_geometry)
            batch.environments.append(env)

        batch_results = asyncio.run(self._runner.run_batch(batch, record_settings=None))

        fitnesses = [
            self._calculate_fitness(
                environment_result.environment_states[0].actor_states[0],
                environment_result.environment_states[-1].actor_states[0],
            )
            for environment_result in batch_results.environment_results
        ]
        return fitnesses

    @staticmethod
    def _calculate_fitness(begin_state: ActorState, end_state: ActorState) -> float:
        # distance traveled on the xy plane
        return math.sqrt(
            (begin_state.position[0] - end_state.position[0]) ** 2
            + ((begin_state.position[1] - end_state.position[1]) ** 2)
        )
