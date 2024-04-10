from pyrr import Quaternion, Vector3

from .._color import Color
from .._module import Module


class Motor(Module):
    """A Motor, made of two distinct parts: the rotor and the frame."""

    def __init__(
        self,
        position: Vector3,
        orientation: Quaternion,
        rotor_size: list[float],  # radius, length
        frame_size: list[float],  # radius, length
        mass: float,
        control_range: list[float],
        clockwise_rotation: bool,
    ):
        """
        Initialize this object.

        :param position: The position to place the motor, reletive to the robots centre (metres).
        :param orientation: Which way the motor is facing as a quaternion.
        :param rotor_size: The size of the cylindrical rotor shape defined by radius and length (metres).
        :param frame_size: The size of the cylindrical frame shape defined by radius and length (metres).
        :param mass: The Modules mass (in kg).
        :param control_range: The upper and lower limit for controlling the motor
        :param clockwise_rotation: Which way the motor is spinning with a torque of 0.1. If True, spin clockwise.
        """
        self._position = position
        self._orientation = orientation
        self._rotor_size = rotor_size
        self._frame_size = frame_size
        self._mass = mass
        self._control_range = control_range
        self._clockwise_rotation = clockwise_rotation

        self._rotor_color = Color(50, 255, 50, 100)
        self._arm_color = Color(255, 50, 50, 100)

        ## TODO: the idea of rotation as a float or right angle doesn't make sense in Module
        super().__init__(0.0, Color(50, 50, 255, 255), {}, [])

    @property
    def position(self) -> Vector3:
        """
        Return the position of the motor in relation to the centre.

        :returns: Position as a Vector3d.
        """
        return self._position

    @property
    def orientation(self) -> Quaternion:
        """
        Return the orientation of the motor in relation to the centre.

        :returns: Orientation as a Quaternion.
        """
        return self._orientation

    @property
    def rotor_size(self) -> list[float]:
        """
        Return the size of the cylindrical rotor shape defined by radius and length (metres).

        :returns: The rotor size as a list of two elements, radius and length.
        """
        return self._rotor_size

    @property
    def frame_size(self) -> list[float]:
        """
        Return the size of the cylindrical frame shape defined by radius and length (metres).

        :returns: The size of the frame as a list of two elements, radius and length.
        """
        return self._frame_size

    @property
    def mass(self) -> float:
        """
        Return the mass of the module (in kg).

        :returns: The mass in kg as a float.
        """
        return self._mass

    @property
    def control_range(self) -> list[float]:
        """
        Return the upper and lower limit for controlling the motor.

        :returns: A list containing two elements, the lower and upper limit respectively.
        """
        return self._control_range

    @property
    def clockwise_rotation(self) -> bool:
        """
        Return which way the motor is spinning with a torque of 0.1.

        :returns: Bool,  If True, the motor is spinning clockwise, else anti-clockwise.
        """
        return self._clockwise_rotation

    @property
    def rotor_color(self) -> Color:
        """
        Return the color of the rotor.

        :returns: The color.
        """
        return self._rotor_color

    @rotor_color.setter
    def rotor_color(self, color: Color) -> None:
        """
        Set the color of the rotor.

        :param color: The color to set the rotor.
        """
        self._rotor_color = color

    @property
    def arm_color(self) -> Color:
        """
        Return the color of the arm.

        :return: The color.
        """
        return self._rotor_color

    @arm_color.setter
    def arm_color(self, color: Color) -> None:
        """
        Set the color of the arm.

        :param color: The color to set the arm.
        """
        self._arm_color = color
