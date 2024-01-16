import numpy as np
from numpy.typing import NDArray

from revolve2.modular_robot.body.base import ActiveHingeSensor, CameraSensor
from revolve2.modular_robot.sensor_state import (
    ActiveHingeSensorState,
    CameraSensorState,
    ModularRobotSensorState,
)

from ._camera_sensor_state_impl import CameraSensorStateImpl


class ModularRobotSensorStateImplV1(ModularRobotSensorState):
    """Implementation of ModularRobotSensorState for v1 robots."""

    _image: NDArray[np.int_]

    def __init__(self, image: NDArray[np.int_]) -> None:
        """
        Initialize this object.

        :param image: The image associated.
        """
        self._image = image

    def get_active_hinge_sensor_state(
        self, sensor: ActiveHingeSensor
    ) -> ActiveHingeSensorState:
        """
        Get sensor states for Hinges.

        :param sensor: The sensor to query.
        :raises NotImplementedError: Always.
        """
        raise NotImplementedError("V1 hardware does not support sensor reading.")

    def get_camera_sensor_state(self, sensor: CameraSensor) -> CameraSensorState:
        """
        Get the current sensor state of the camera sensor.

        :param sensor: The camera sensor.
        :return: The sensor state.
        """
        return CameraSensorStateImpl(self._image)
