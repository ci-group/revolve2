import numpy as np
import numpy.typing as npt

from ..._aerial_robot_control_interface import AerialRobotControlInterface
from ...body.base import Motor
from ...sensor_state import AerialRobotSensorState
from .._brain_instance import BrainInstance


class BrainDefaultInstance(BrainInstance):
    """
    Default brain. Simple input-output to motors. State defines the inputs
    """

    def __init__(
        self,
        initial_state: npt.NDArray[np.float_],
        output_mapping: list[tuple[int, Motor]],
    ) -> None:
        """
        Initialize this object.

        :param initial_state: The initial state of the motors.
        :param output_mapping: Marks neurons as controller outputs and map them to the correct motor.
        """
        assert all([i >= 0 and i < len(initial_state) for i, _ in output_mapping])

        self._state = initial_state
        self._output_mapping = output_mapping

    def control(
        self,
        dt: float,
        sensor_state: AerialRobotSensorState,
        control_interface: AerialRobotControlInterface,
    ) -> None:
        """
        Control the aerial robot.

        Sets the motor targets to the values in the state array as defined by the mapping provided in the constructor.

        :param dt: Elapsed seconds since last call to this function.
        :param sensor_state: Interface for reading the current sensor state.
        :param control_interface: Interface for controlling the robot.
        """

        # Set motor targets to match newly calculated state.
        for state_index, motor in self._output_mapping:
            control_interface.set_motor_target(
                motor, float(self._state[state_index]) * motor.range
            )
