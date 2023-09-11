"""Evaluator class."""

import asyncio
import math

import numpy as np
import numpy.typing as npt
from revolve2.actor_controllers.cpg import CpgNetworkStructure
from revolve2.ci_group import fitness_functions, terrains
from revolve2.ci_group.simulation import create_batch_multiple_isolated_robots_standard
from revolve2.modular_robot import (
    Body,
    ModularRobot,
    get_body_states_multiple_isolated_robots,
)
from revolve2.modular_robot.brains import BrainCpgNetworkStatic
from revolve2.simulation import Terrain
from revolve2.simulation.running import Runner
from revolve2.simulators.mujoco import LocalRunner


class Evaluator:
    """Provides evaluation of robots."""

    _runner: Runner
    _terrain: Terrain
    _cpg_network_structure: CpgNetworkStructure
    _body: Body

    def __init__(
        self,
        headless: bool,
        num_simulators: int,
        cpg_network_structure: CpgNetworkStructure,
        body: Body,
    ) -> None:
        """
        Initialize this object.

        :param headless: `headless` parameter for the physics runner.
        :param num_simulators: `num_simulators` parameter for the physics runner.
        :param cpg_network_structure: Cpg structure for the brain.
        :param body:Modular body of the robot.
        """
        self._runner = LocalRunner(headless=headless, num_simulators=num_simulators)
        self._terrain = terrains.flat()
        self._cpg_network_structure = cpg_network_structure
        self._body = body

    def evaluate(
        self,
        solutions: list[npt.NDArray[np.float_]],
    ) -> npt.NDArray[np.float_]:
        """
        Evaluate multiple robots.

        Fitness is the distance traveled on the xy plane.

        :param solutions: Solutions to evaluate.
        :returns: Fitnesses of the solutions.
        """
        # Create robots from the brain parameters.
        robots = [
            ModularRobot(
                self._body,
                BrainCpgNetworkStatic.create_simple(
                    params=params,
                    cpg_network_structure=self._cpg_network_structure,
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
            [self._body for _ in solutions], results
        )
        xy_displacements = [
            fitness_functions.xy_displacement(body_state_begin, body_state_end)
            for body_state_begin, body_state_end in body_states
        ]
        return np.array(xy_displacements)
