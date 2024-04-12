from ..._modular_robot_control_interface import ModularRobotControlInterface
from ...sensor_state._modular_robot_sensor_state import ModularRobotSensorState
from .._brain_instance import BrainInstance


class BrainDummyInstance(BrainInstance):
    """A brain that does nothing."""

    def control(
        self,
        dt: float,
        sensor_state: ModularRobotSensorState,
        control_interface: ModularRobotControlInterface,
    ) -> None:
        """
        Control the modular robot.

        Sets the active hinge targets to the values in the state array as defined by the mapping provided in the constructor.

        :param dt: Elapsed seconds since last call to this function.
        :param sensor_state: Interface for reading the current sensor state.
        :param control_interface: Interface for controlling the robot.
        """
