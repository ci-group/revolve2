"""Evaluator class."""

from database_components import Genotype
import datetime
from revolve2.experimentation.evolution.abstract_elements import Evaluator as Eval
from revolve2.modular_robot_simulation import (
    ModularRobotScene,
    Terrain,
    simulate_scenes,
)
from revolve2.simulators.mujoco_simulator import LocalSimulator

from tasks import gait_learning, turning_in_place, uphill_locomotion
import terrains
from revolve2.standards.simulation_parameters import make_standard_batch_parameters
import matplotlib.pyplot as plt
plt.switch_backend('Qt5Agg')

class Evaluator(Eval):
    """Provides evaluation of robots."""

    _simulator: LocalSimulator
    _terrain: Terrain

    def __init__(
        self,
        headless: bool,
        num_simulators: int,
        terrain: str = "flat",
        terrain_params: str = "",
        task: str = "gait_learning",
        fitness_function: str ="xy_displacement"
    ) -> None:
        """
        Initialize this object.

        :param headless: `headless` parameter for the physics simulator.
        :param num_simulators: `num_simulators` parameter for the physics simulator.
        """
        self._simulator = LocalSimulator(
            headless=headless, num_simulators=num_simulators
        )
        self._terrain = eval(f"terrains.{terrain}")
        if terrain_params != "":
            self._terrain_params = eval(terrain_params)
            print("terrain_params are : ", self._terrain_params)
            print("terrain params_type is: ", type(self._terrain_params))
            
        self._task = eval(task)
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

        # Get the fitness function from the correct task file.
        fitness_function = getattr(self._task, self._fitness_function)
               
        # Need the entirety of the trajectory to evaluate turning in place.
        if fitness_function is turning_in_place.circular_trajectory:
            fitnesses = [
                fitness_function(
                    [states[i].get_modular_robot_simulation_state(robot)
                      for i in range(len(states))],
                        radius=.15)
                for robot, states in zip(robots, scene_states) 
            ]

        elif fitness_function is turning_in_place.turn_360_fitness:
            fitnesses = [
                fitness_function(
                    [states[i].get_modular_robot_simulation_state(robot)
                      for i in range(len(states))]
                      )
                for robot, states in zip(robots, scene_states) 
            ]

        elif fitness_function is uphill_locomotion.uphill_displacement:
            fitnesses = [
                fitness_function(states[0].get_modular_robot_simulation_state(robot),
                                states[-1].get_modular_robot_simulation_state(robot),
                                tilt_angle=float(self._terrain_params['tilt_angle']),
                                tilt_direction=eval(self._terrain_params['tilt_direction']))
                for robot, states in zip(robots, scene_states)
            ]
            
        # Initial and final states are enough for the other tasks.
        else:
            fitnesses = [
                fitness_function(states[0].get_modular_robot_simulation_state(robot),
                                states[-1].get_modular_robot_simulation_state(robot))
                for robot, states in zip(robots, scene_states)
            ]

        # visualize path of the robot
        if len(population) == 1:
            to_visualize = [[states[i].get_modular_robot_simulation_state(robot) for i in range(len(states))]
                    for robot, states in zip(robots, scene_states)]
            visualize_path(to_visualize[0])

        return fitnesses

def visualize_path(states):
    x = [state.get_pose().position.x for state in states]
    y = [state.get_pose().position.y for state in states]
    timestamp = datetime.datetime.now()

    plt.plot(x, y)
    plt.savefig(f'gui/resources/figures/path_{timestamp}.png')
    plt.show()
    plt.close()
