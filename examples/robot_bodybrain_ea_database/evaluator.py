"""Evaluator class."""

import asyncio
from typing import List

from revolve2.core.modular_robot import (
    ModularRobot,
    get_body_states_multiple_isolated_robots,
)
from revolve2.core.physics import Terrain
from revolve2.core.physics.running import Runner
from revolve2.runners.mujoco import LocalRunner
from revolve2.standard_resources import fitness_functions, terrains
from revolve2.standard_resources.simulation import (
    create_batch_multiple_isolated_robots_standard,
)


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
        robots: List[ModularRobot],
    ) -> List[float]:
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