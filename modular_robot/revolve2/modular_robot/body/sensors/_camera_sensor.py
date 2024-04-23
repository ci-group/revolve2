from pyrr import Quaternion, Vector3

from ._sensor import Sensor


class CameraSensor(Sensor):
    """A camera for the Modular Robot."""

    _camera_size: tuple[int, int]

    def __init__(
        self,
        position: Vector3,
        orientation: Quaternion = Quaternion(),
        camera_size: tuple[int, int] = (50, 50),
    ) -> None:
        """
        Initialize the Camera Sensor.

        Note that the camera_size can have a significant impact on performance.
        For evolution related work stick to 10x10 for fast results.

        :param position: The position of the camera.
        :param orientation: The rotation of the camera.
        :param camera_size: The size of the camera image.
        """
        super().__init__(orientation, position)
        self._camera_size = camera_size

    @property
    def camera_size(self) -> tuple[int, int]:
        """
        Get the size of the camera.

        :return: The camera size.
        """
        return self._camera_size
