from pyrr import Quaternion, Vector3

from .._color import Color
from .._module import Module

class Motor(Module):
    """A Motor."""

    position: float
    orientation: float
    rotor_bounding_box: Vector3
    frame_bounding_box: Vector3
    mass : float
    ctrlrange: list[float]
    gear: float

    def __init__(
        self,
        position: Vector3,
        orientation: Quaternion, 
        rotor_bounding_box: Vector3,
        frame_bounding_box: Vector3,
        mass : float,
        ctrlrange: list[float],
        gear: float
    ):
        """
        Initialize this object.
        """
        self._position = position
        self._orientation = orientation
        self._rotor_bounding_box = rotor_bounding_box
        self._frame_bounding_box = frame_bounding_box
        self._mass = mass
        self._ctrlrange = ctrlrange
        self._gear = gear

        self.num_attachments = 0
        attachment_points = []
        sensors = []
        super().__init__(0.0, Color(50, 50, 255, 255), attachment_points, sensors)

    def add_attachment(self, module):
        self.set_child(module, self.num_attachments)
        self.num_attachments += 1

    @property
    def position(self):
        return self._position
    
    @property
    def orientation(self):
        return self._orientation
    
    @property
    def rotor_bounding_box(self):
        return self._rotor_bounding_box

    @property
    def frame_bounding_box(self):
        return self._frame_bounding_box
    
    @property
    def mass(self):
        return self._mass
    
    @property
    def ctrlrange(self):
        return self._ctrlrange
    
    @property
    def gear(self):
        return self._gear