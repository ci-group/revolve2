import math

from pyrr import Quaternion, Vector3

from .._color import Color
from .._module import Module

class Core(Module):
    """The core module of a modular robot."""

    def __init__(
        self,
        rotation: float,
        mass: float,
        bounding_box: Vector3,
    ):
        """
        Initialize this object.

        :param rotation: The Modules rotation.
        :param mass: The Modules mass (in kg).
        :param bounding_box: The bounding box. Vector3 with sizes of bbox in x,y,z dimension (m). Sizes are total length, not half length from origin.
        """
        self._mass = mass
        self._bounding_box = bounding_box

        super().__init__(rotation, Color(255, 50, 50, 255))

    @property
    def mass(self) -> float:
        """
        Get the mass of the Core (in kg).

        :return: The value.
        """
        return self._mass

    @property
    def bounding_box(self) -> Vector3:
        """
        Get the bounding box.

        Sizes are total length, not half length from origin.
        :return: Vector3 with sizes of bbox in x,y,z dimension (m).
        """
        return self._bounding_box
