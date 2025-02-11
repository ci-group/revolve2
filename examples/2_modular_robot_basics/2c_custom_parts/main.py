"""Main script for the example."""

from pyrr import Vector3

from revolve2.experimentation.logging import setup_logging
from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body import RightAngles
from revolve2.modular_robot.body.v2 import ActiveHingeV2, BodyV2, BrickV2, CoreV2
from revolve2.modular_robot.brain.dummy import BrainDummy
from revolve2.modular_robot_simulation import test_robot
from revolve2.simulators.mujoco_simulator import LocalSimulator
from revolve2.standards import terrains
from revolve2.standards.simulation_parameters import make_standard_batch_parameters

""" Below we create a custom version of the ActiveHingeV2. 
    By pressing shift+right click on the ActiveHingeV2 class in the IDE you can see where it is defined.
    Changing the parameters there will change it for all instances of ActiveHingeV2, which is not what we want.
    Therefore we create a custom version of the ActiveHingeV2 and change the parameters there."""


class CustomActiveHingeV2(ActiveHingeV2):
    """Custom ActiveHinge for parameter testing."""

    def __init__(self, rotation: float | RightAngles):
        """Initialize with custom parameters.

        :param rotation: The Modules rotation.

        """
        super().__init__(rotation=rotation)
        # Modify parameters here to experiment
        self._frame_bounding_box = Vector3([0.036, 0.104, 0.033])  # Double size
        self._servo1_bounding_box = Vector3([0.1025, 0.1024, 0.040])  # Double size
        self._servo2_bounding_box = Vector3([0.004, 0.104, 0.104])  # Double size


class CustomBrickV2(BrickV2):
    """Custom Brick for parameter testing."""

    def __init__(self, rotation: float | RightAngles):
        """Initialize with custom parameters.

        :param rotation: The Modules rotation.

        """
        super().__init__(rotation=rotation)
        # Modify parameters here to experiment
        self._mass = 0.100  # Double mass
        self._bounding_box = Vector3([0.130, 0.130, 0.130])  # Double size


class CustomCoreV2(CoreV2):
    """Custom Core for parameter testing."""

    def __init__(self, rotation: float | RightAngles):
        """Initialize with custom parameters.

        :param rotation: The Modules rotation.

        """
        super().__init__(rotation=rotation)
        # Modify parameters here to experiment
        self._mass = 0.200  # Double mass
        self._bounding_box = Vector3([0.130, 0.130, 0.130])  # Double size


def make_visualization_body() -> BodyV2:
    """
    Create a body with all components for visualization.

    :returns: The created body.
    """
    body = BodyV2()

    # Attach components in different configurations to visualize all parts
    hinge_left = CustomActiveHingeV2(RightAngles.DEG_0)
    body.core_v2.front_face.bottom = hinge_left
    hinge_left.attachment = CustomBrickV2(RightAngles.DEG_0)

    # Add a chain of components to see how they connect
    hinge = CustomActiveHingeV2(RightAngles.DEG_0)
    body.core_v2.back_face.bottom = hinge
    hinge.attachment = CustomBrickV2(RightAngles.DEG_0)

    return body


def main() -> None:
    """Run the visualization."""
    setup_logging()

    body = make_visualization_body()

    # Create dummy brain (no movement) for better visualization
    brain = BrainDummy()
    robot = ModularRobot(body, brain)

    # Set up simulator with custom viewer for better visualization
    simulator = LocalSimulator(
        viewer_type="custom",
        headless=False,
    )

    # Configure sim params
    batch_parameters = make_standard_batch_parameters()
    batch_parameters.simulation_time = (
        10  # Short sim time since we just want to view it
    )
    batch_parameters.simulation_timestep = (
        0.005  # Smaller timestep for smoother visualization
    )

    # Test the robot using the test_robot utility
    test_robot(
        robot=robot,
        terrain=terrains.flat(),
        simulator=simulator,
        batch_parameters=batch_parameters,
    )


if __name__ == "__main__":
    main()
