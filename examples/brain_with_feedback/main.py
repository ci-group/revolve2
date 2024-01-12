"""Main script for the example."""

import logging

from revolve2.ci_group import modular_robots_v1, terrains
from revolve2.ci_group.simulation_parameters import make_standard_batch_parameters
from revolve2.experimentation.logging import setup_logging
from revolve2.modular_robot import ModularRobot, ModularRobotControlInterface
from revolve2.modular_robot.body.base import ActiveHinge, ActiveHingeSensor, IMUSensor
from revolve2.modular_robot.brain import Brain, BrainInstance
from revolve2.modular_robot.sensor_state import ModularRobotSensorState
from revolve2.modular_robot_simulation import ModularRobotScene, simulate_scenes
from revolve2.simulators.mujoco_simulator import LocalSimulator


class ANNBrainInstance(BrainInstance):
    """ANN brain instance."""

    active_hinges: list[ActiveHinge]
    imu_sensor: IMUSensor

    def __init__(self, active_hinges: list[ActiveHinge], imu_sensor: IMUSensor) -> None:
        """
        Initialize the Object.

        :param active_hinges: The active hinges to control.
        :param imu_sensor: The IMU sensor.
        """
        self.active_hinges = active_hinges
        self.imu_sensor = imu_sensor

    def control(
        self,
        dt: float,
        sensor_state: ModularRobotSensorState,
        control_interface: ModularRobotControlInterface,
    ) -> None:
        """
        Control the modular robot.

        :param dt: Elapsed seconds since last call to this function.
        :param sensor_state: Interface for reading the current sensor state.
        :param control_interface: Interface for controlling the robot.
        """
        # Get the sensors from the active hinges
        sensors = [
            active_hinge.sensor
            for active_hinge in self.active_hinges
            if active_hinge.sensor is not None
        ]
        assert len(sensors) == len(
            self.active_hinges
        ), "One of the active hinges does not have a sensor set."

        # Get the current angular positions of the active hinges
        current_positions = [
            sensor_state.get_active_hinge_sensor_state(sensor).position
            for sensor in sensors
        ]
        logging.info(current_positions)

        # Get the imu state
        imu_state = sensor_state.get_imu_sensor_state(self.imu_sensor)
        logging.info(imu_state.orientation)
        logging.info(imu_state.angular_rate)
        logging.info(imu_state.specific_force)

        # Here you can implement your controller.
        # The current controller does nothing except for always settings the joint positions to 0.5.
        for active_hinge, sensor in zip(self.active_hinges, sensors):
            target = 0.5
            control_interface.set_active_hinge_target(active_hinge, target)


class ANNBrain(Brain):
    """The ANN brain."""

    active_hinges: list[ActiveHinge]
    imu_sensor: IMUSensor

    def __init__(self, active_hinges: list[ActiveHinge], imu_sensor: IMUSensor) -> None:
        """
        Initialize the Object.

        :param active_hinges: The active hinges to control.
        :param imu_sensor: The IMU sensor.
        """
        self.active_hinges = active_hinges
        self.imu_sensor = imu_sensor

    def make_instance(self) -> BrainInstance:
        """
        Create an instance of this brain.

        :returns: The created instance.
        """
        return ANNBrainInstance(
            active_hinges=self.active_hinges, imu_sensor=self.imu_sensor
        )


def main() -> None:
    """Run the simulation."""
    # Set up logging.
    setup_logging()

    # Create a body for the robot.
    body = modular_robots_v1.gecko_v1()

    # Add sensors to each active hinge that measure the current angle of the hinge.
    active_hinges = body.find_modules_of_type(ActiveHinge)
    for active_hinge in active_hinges:
        active_hinge.sensor = ActiveHingeSensor()

    body.core._imu_sensor = IMUSensor()

    # Create a brain for the robot.
    active_hinges = body.find_modules_of_type(ActiveHinge)
    brain = ANNBrain(active_hinges=active_hinges, imu_sensor=body.core._imu_sensor)

    # Combine the body and brain into a modular robot.
    robot = ModularRobot(body, brain)

    # Create the scene.
    scene = ModularRobotScene(terrain=terrains.flat())
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
