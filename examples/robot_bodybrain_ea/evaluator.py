"""Evaluator class."""

import asyncio

from revolve2.ci_group import fitness_functions, terrains
from revolve2.ci_group.simulation import create_batch_multiple_isolated_robots_standard
from revolve2.modular_robot import (
    ModularRobot,
    get_body_states_multiple_isolated_robots,
)
from revolve2.simulation import Terrain
from revolve2.simulation.running import Runner
from revolve2.simulators.mujoco import LocalRunner


class Evaluator:
    """Provides evaluation of robots."""

    _runner: Runner
    _terrain: Terrain

    def __init__(
        self,
        headless: bool,
        num_simulators: int,
    ) -> None:
        """
        Initialize this object.

        :param headless: `headless` parameter for the physics runner.
        :param num_simulators: `num_simulators` parameter for the physics runner.
        """
        self._runner = LocalRunner(headless=headless, num_simulators=num_simulators)
        self._terrain = terrains.flat()

    def evaluate(
        self,
        robots: list[ModularRobot],
    ) -> list[float]:
        """
        Evaluate multiple robots.

        Fitness is the distance traveled on the xy plane.

        :param robots: The robots to simulate.
        :returns: Fitnesses of the robots.
        """
        # Simulate the robots and process the results.
        batch = create_batch_multiple_isolated_robots_standard(
            robots, [self._terrain for _ in robots]
        )
        results = asyncio.run(self._runner.run_batch(batch))

        body_states = get_body_states_multiple_isolated_robots(
            [robot.body for robot in robots], results
        )
        xy_displacements = [
            fitness_functions.xy_displacement(body_state_begin, body_state_end)
            for body_state_begin, body_state_end in body_states
        ]
        return xy_displacements
