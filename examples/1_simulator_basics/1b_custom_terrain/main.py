"""Main script for the example."""

import math

from pyrr import Quaternion, Vector3
from revolve2.ci_group.modular_robots_v2 import gecko_v2
from revolve2.ci_group.simulation_parameters import make_standard_batch_parameters
from revolve2.experimentation.logging import setup_logging
from revolve2.experimentation.rng import make_rng_time_seed
from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.brain.cpg import BrainCpgNetworkNeighborRandom
from revolve2.modular_robot_simulation import (
    ModularRobotScene,
    Terrain,
    simulate_scenes,
)
from revolve2.simulation.scene import AABB, Color, Pose
from revolve2.simulation.scene.geometry import GeometryBox, GeometryPlane
from revolve2.simulation.scene.geometry.textures import MapType
from revolve2.simulators.mujoco_simulator import LocalSimulator
from revolve2.simulators.mujoco_simulator.textures import Checker, Flat, Gradient


def make_custom_terrain() -> Terrain:
    """
    Create a custom terrain.

    :returns: The created terrain.
    """
    # A terrain is a collection of static geometries.
    # Here we create a simple terrain uses some boxes.
    return Terrain(
        static_geometry=[
            GeometryPlane(
                pose=Pose(position=Vector3(), orientation=Quaternion()),
                mass=0.0,
                size=Vector3([20.0, 20.0, 0.0]),
                texture=Checker(
                    primary_color=Color(170, 170, 180, 255),
                    secondary_color=Color(150, 150, 150, 255),
                    map_type=MapType.MAP2D,
                ),
            ),
            GeometryBox(
                pose=Pose(position=Vector3([1.0, 0.0, 0.1]), orientation=Quaternion()),
                mass=0.0,
                texture=Flat(primary_color=Color(0, 255, 0, 255)),
                aabb=AABB(size=Vector3([0.5, 0.5, 0.2])),
            ),
            GeometryBox(
                pose=Pose(
                    position=Vector3([-0.8, 0.4, 0.125]), orientation=Quaternion()
                ),
                mass=0.0,
                texture=Gradient(
                    primary_color=Color(0, 200, 100, 255),
                    secondary_color=Color(0, 100, 200, 255),
                ),
                aabb=AABB(size=Vector3([0.5, 0.5, 0.25])),
            ),
            GeometryBox(
                pose=Pose(
                    position=Vector3([-0.8 + 0.38, 0.3, 0.125]),
                    orientation=Quaternion.from_eulers([0.0, math.pi / 4.0, 0.0]),
                ),
                mass=0.0,
                texture=Flat(primary_color=Color(50, 80, 180, 255)),
                aabb=AABB(size=Vector3([0.5, 0.4, 0.02])),
            ),
            GeometryBox(
                pose=Pose(position=Vector3([-0.1, 0.9, 0.5]), orientation=Quaternion()),
                mass=0.0,
                texture=Flat(
                    primary_color=Color(100, 0, 100, 255),
                    base_color=Color(255, 255, 255, 100),
                ),
                aabb=AABB(size=Vector3([0.2, 0.2, 1.0])),
            ),
        ]
    )


def main() -> None:
    """Run the simulation."""
    # Set up logging.
    setup_logging()

    # Set up the random number generator.
    rng = make_rng_time_seed()

    # Create a robot
    body = gecko_v2()
    brain = BrainCpgNetworkNeighborRandom(body=body, rng=rng)
    robot = ModularRobot(body, brain)

    # Create the scene.
    scene = ModularRobotScene(terrain=make_custom_terrain())
    scene.add_robot(robot)

    # Simulate the scene.
    simulator = LocalSimulator()
    simulate_scenes(
        simulator=simulator,
        batch_parameters=make_standard_batch_parameters(),
        scenes=scene,
    )


if __name__ == "__main__":
    main()
