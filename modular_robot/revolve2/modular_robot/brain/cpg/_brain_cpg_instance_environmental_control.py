from ._brain_cpg_instance import BrainCpgInstance
from ..._modular_robot_control_interface import ModularRobotControlInterface
from ...sensor_state import ModularRobotSensorState
from ...body.base import CameraSensor
import numpy as np


class BrainCpgInstanceEnvironmentalControl(BrainCpgInstance):
    """A Brain instance for the environmental controlled cpg."""

    _n: int = 7

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
        # Integrate ODE to obtain new state.
        self._state = self._rk45(self._state, self._weight_matrix, dt)
        image = sensor_state.get_camera_sensor_state(CameraSensor()).image  # TODO: make this proper

        image = np.flip(image)  # flip the image because its axis are inverted

        # we use color filters to find the next target point
        green_filter = image[:, :, 1] < 100
        red_filter = image[:, :, 0] > 100
        blu_filter = image[:, :, 2] < 100
        coordinates = np.where(green_filter & red_filter & blu_filter)
        x_pos = 0
        if coordinates[1].shape[0] > 0:
            x_pos = np.mean(coordinates[1])
        picture_width = image.shape[1]
        theta = (picture_width - x_pos) - (picture_width / 2)
        g = (((picture_width/2) - abs(theta)) / (picture_width / 2)) ** self._n

        # Set active hinge targets to match newly calculated state.
        for state_index, active_hinge in self._output_mapping:
            control_interface.set_active_hinge_target(
                active_hinge, (float(self._state[state_index]) * active_hinge.range) * g
            )
