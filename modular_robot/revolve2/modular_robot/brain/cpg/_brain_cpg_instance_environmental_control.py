from ._brain_cpg_instance import BrainCpgInstance
from ..._modular_robot_control_interface import ModularRobotControlInterface
from ...sensor_state import ModularRobotSensorState
from ...body.base import CameraSensor
import numpy as np
from tempfile import NamedTemporaryFile
from PIL import Image
from subprocess import call
import os
from numpy.typing import NDArray



class BrainCpgInstanceEnvironmentalControl(BrainCpgInstance):
    """A Brain instance for the environmental controlled cpg."""

    _n: int = 5

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
        image = self.__get_image()

        image = np.flip(image)  # flip the image because its axis are inverted

        # we use color filters to find the next target point
        red_filter = image[:, :, 0] > 100
        green_filter = image[:, :, 1] < 100
        blue_filter = image[:, :, 2] < 100
        coordinates = np.where(green_filter & red_filter & blue_filter)
        x_pos = 0
        if coordinates[0].shape[0] > 0:
            x_pos = np.mean(coordinates[0])
        picture_width = image.shape[1]
        theta = (picture_width - x_pos) - (picture_width / 2)
        print("theta: ", theta)
        g = (((picture_width/2) - abs(theta)) / (picture_width / 2)) ** self._n
        print("g: ", g)

        # Set active hinge targets to match newly calculated state.
        for i, (state_index, active_hinge) in enumerate(self._output_mapping):
            if 7 > i > 3 and g >= 0:
                control_interface.set_active_hinge_target(
                    active_hinge, float((float(self._state[state_index]) * active_hinge.range) * g)
                )
            else:  # Center axis joints are not affected
                control_interface.set_active_hinge_target(
                    active_hinge, (float(self._state[state_index]) * active_hinge.range)
                )
        print("state: ", self._state)

    def __get_image(self):
        with NamedTemporaryFile(suffix=".jpeg") as tmp_file:
            file_name = tmp_file.name
            call(f"rpicam-jpeg -n -t 10 -o {file_name} --width 400 --height 400", shell=True)
            pil_image = Image.open(file_name)
        image: NDArray[np.int_] = np.asarray(pil_image)
        return image