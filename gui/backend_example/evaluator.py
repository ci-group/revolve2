"""Evaluator class."""

from database_components import Genotype

from revolve2.experimentation.evolution.abstract_elements import Evaluator as Eval
from revolve2.modular_robot_simulation import (
    ModularRobotScene,
    Terrain,
    simulate_scenes,
)
from revolve2.simulators.mujoco_simulator import LocalSimulator

import fitness_functions
import terrains
from revolve2.standards.simulation_parameters import make_standard_batch_parameters


class Evaluator(Eval):
    """Provides evaluation of robots."""

    _simulator: LocalSimulator
    _terrain: Terrain

    def __init__(
        self,
        headless: bool,
        num_simulators: int,
        terrain="flat",
        fitness_function="xy_displacement"
    ) -> None:
        """
        Initialize this object.

        :param headless: `headless` parameter for the physics simulator.
        :param num_simulators: `num_simulators` parameter for the physics simulator.
        """
        self._simulator = LocalSimulator(
            headless=headless, num_simulators=num_simulators
        )
        self._terrain = eval("terrains."+terrain+"()")
        self._fitness_function = fitness_function

    def evaluate(
        self,
        population: list[Genotype],
    ) -> list[float]:
        """
        Evaluate multiple robots.

        Fitness is the distance traveled on the xy plane.

        :param population: The robots to simulate.
        :returns: Fitnesses of the robots.
        """
        robots = [genotype.develop() for genotype in population]
        # Create the scenes.
        scenes = []
        for robot in robots:
            scene = ModularRobotScene(terrain=self._terrain)
            scene.add_robot(robot)
            scenes.append(scene)

        # Simulate all scenes.
        scene_states = simulate_scenes(
            simulator=self._simulator,
            batch_parameters=make_standard_batch_parameters(),
            scenes=scenes,
        )

        # Calculate the fitnesses.
        fitness_function = getattr(fitness_functions, self._fitness_function)
        fitnesses = [
            fitness_function(states[0].get_modular_robot_simulation_state(robot),
                             states[-1].get_modular_robot_simulation_state(robot))
            for robot, states in zip(robots, scene_states)
        ]

        return fitnesses
