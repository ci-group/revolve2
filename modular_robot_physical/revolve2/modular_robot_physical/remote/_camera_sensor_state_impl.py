import numpy as np
from numpy.typing import NDArray
from revolve2.modular_robot.sensor_state import CameraSensorState


class CameraSensorStateImpl(CameraSensorState):
    """CameraSensorState implementation for physical robots."""

    _image: NDArray[np.uint8]

    def __init__(self, image: NDArray[np.uint8]) -> None:
        """
        Initialize this object.

        :param image: The current image.
        """
        self._image = image

    @property
    def image(self) -> NDArray[np.uint8]:
        """
        Get the current image.

        :returns: The image.
        """
        return self._image
