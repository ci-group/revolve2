import math

from pyrr import Quaternion, Vector3

from .._color import Color
from .._module import Module

class DroneCore(Module):
    """The core module of a modular robot."""

    def __init__(
        self,
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
        self.num_attachments = 0
        attachment_points = []
        sensors = []
        ## TODO: Remove rotation from inherited module class
        super().__init__(0.0, Color(255, 50, 50, 255), attachment_points, sensors)

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

    def add_attachment(self, module):
        self.set_child(module, self.num_attachments)
        self.num_attachments += 1
        