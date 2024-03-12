from pyrr import Vector3

from ._sensor import Sensor


class CameraSensor(Sensor):
    """A camera for the Modular Robot."""

    _position: Vector3
    _camera_size: tuple[int, int]

    def __init__(
        self,
        position: Vector3,
        rotation: float = 0.0,
        camera_size: tuple[int, int] = (50, 50),
    ) -> None:
        """
        Initialize the Camera Sensor.

        Note that the camera_size can have a significant impact on performance.
        For evolution related work sticked to 10x10 for sei fast reulst.

        :param position: The position of the camera.
        :param rotation: The rotation of the camera.
        :param camera_size: The size of the camera image.
        """
        super().__init__(rotation)
        self._position = position
        self._camera_size = camera_size

    @property
    def position(self) -> Vector3:
        """
        Get the position of the camera.

        :return: The position.
        """
        return self._position

    @property
    def camera_size(self) -> tuple[int, int]:
        """
        Get the size of the camera.

        :return: The camera size.
        """
        return self._camera_size
