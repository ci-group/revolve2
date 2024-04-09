from pyrr import Quaternion, Vector3

from .._color import Color
from .._module import Module

class Motor(Module):
    """A Motor."""

    position: float
    orientation: float
    rotor_size: list[float] # radius, length
    frame_size: list[float] # radius, length
    mass : float
    control_range: list[float]
    gear: float

    def __init__(
        self,
        position: Vector3,
        orientation: Quaternion, 
        rotor_size: list[float], # radius, length
        frame_size: list[float], # radius, length
        mass : float,
        control_range: list[float],
        gear: float
    ):
        """
        Initialize this object.
        """
        self._position = position
        self._orientation = orientation
        self._rotor_size = rotor_size
        self._frame_size = frame_size
        self._mass = mass
        self._control_range = control_range
        self._gear = gear

        self.num_attachments = 0
        attachment_points = []
        sensors = []
        self._rotor_color = Color(50, 255, 50, 100)
        self._arm_color = Color(255, 50, 50, 100)

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
    def rotor_size(self):
        return self._rotor_size

    @property
    def frame_size(self):
        return self._frame_size
    
    @property
    def mass(self):
        return self._mass
    
    @property
    def control_range(self):
        return self._control_range
    
    @property
    def gear(self):
        return self._gear
    
    @property
    def rotor_color(self) -> Color:
        return self._rotor_color

    @rotor_color.setter
    def rotor_color(self, color: Color) -> None:
        self._rotor_color = color

    @property
    def arm_color(self) -> Color:
        return self._rotor_color

    @arm_color.setter
    def arm_color(self, color: Color) -> None:
        self._arm_color = color