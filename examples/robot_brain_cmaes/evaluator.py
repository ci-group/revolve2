"""Evaluator class."""

import asyncio
import math
from typing import List

import numpy as np
import numpy.typing as npt
from revolve2.actor_controllers.cpg import CpgNetworkStructure
from revolve2.core.modular_robot import (
    Body,
    ModularRobot,
    get_body_states_multiple_isolated_robots,
)
from revolve2.core.modular_robot.brains import BrainCpgNetworkStatic
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
    _simulation_time: int
    _sampling_frequency: int
    _control_frequency: int

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
        body: Body,
        cpg_network_structure: CpgNetworkStructure,
        solutions: List[npt.NDArray[np.float_]],
    ) -> npt.NDArray[np.float_]:
        """
        Evaluate multiple robots.

        Fitness is the distance traveled on the xy plane.

        :param body: Modular body of the robot.
        :param cpg_network_structure: cpg structure for the brain.
        :param solutions: Solutions to evaluate.
        :returns: Fitnesses of the solutions.
        """
        # Create robots from the brain parameters.
        robots = [
            ModularRobot(
                body,
                BrainCpgNetworkStatic.create_simple(
                    params=params,
                    cpg_network_structure=cpg_network_structure,
                    initial_state_uniform=math.pi / 2.0,
                    dof_range_uniform=1.0,
                ),
            )
            for params in solutions
        ]

        # Next, run the simulation and process the results.
        batch = create_batch_multiple_isolated_robots_standard(
            robots, [self._terrain for _ in solutions]
        )
        results = asyncio.run(self._runner.run_batch(batch))

        body_states = get_body_states_multiple_isolated_robots(
            [body for _ in solutions], results
        )
        xy_displacements = [
            fitness_functions.xy_displacement(body_state_begin, body_state_end)
            for body_state_begin, body_state_end in body_states
        ]
        return np.array(xy_displacements)
