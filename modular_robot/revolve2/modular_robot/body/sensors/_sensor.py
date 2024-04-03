import uuid
from abc import ABC

from pyrr import Vector3


class Sensor(ABC):
    """An abstract Sensor Class."""

    _uuid: uuid.UUID
    _rotation: float
    _position: Vector3

    def __init__(self, rotation: float, position: Vector3 = Vector3([0, 0, 0])) -> None:
        """
        Initialize the sensor.

        :param rotation: The rotation of the sensor.
        :param position: The position of the sensor.
        """
        self._rotation = rotation
        self._uuid = uuid.uuid1()
        self._position = position

    @property
    def uuid(self) -> uuid.UUID:
        """
        Get the uuid of the sensor.

        :return: The uuid.
        """
        return self._uuid

    @property
    def rotation(self) -> float:
        """
        Return the rotation of the sensor.

        :return: The rotation.
        """
        return self._rotation

    @property
    def position(self) -> Vector3:
        """
        Get the position of the sensor on a module.

        :return: The position.
        """
        return self._position
