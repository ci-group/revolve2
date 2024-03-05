from pyrr import Vector3

from revolve2.simulation.scene.vector2 import Vector2

from ._sensor import Sensor


class CameraSensor(Sensor):
    """A camera for the Modular Robot."""

    _position: Vector3
    _camera_size: Vector2

    def initialize(
        self,
        position: Vector3,
        rotation: float = 0.0,
        camera_size: Vector2 = Vector2([200, 200]),
    ) -> None:
        """
        Initialize the Camera Sensor.

        :param position: The position of the camera.
        :param camera_size: The size of the camera image.
        :param rotation: The rotation of the camera.
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
    def camera_size(self) -> Vector2:
        """
        Get the size of the camera.

        :return: The camera size.
        """
        return self._camera_size
