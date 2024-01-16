import numpy as np
from numpy.typing import NDArray

from revolve2.modular_robot.sensor_state import CameraSensorState


class CameraSensorStateImpl(CameraSensorState):
    """CameraSensorState implementation for physical robots."""

    _image: NDArray[np.int_]

    def __init__(self, image: NDArray[np.int_]) -> None:
        """
        Initialize this object.

        :param image: The current view of the camera.
        """
        self._image = image

    @property
    def image(self) -> NDArray[np.int_]:
        """
        Get the image taken by the camera.

        :returns: The image.
        """
        return self._image
