from dataclasses import dataclass

from pyrr import Vector3
from revolve2.simulation.actor._color import Color


@dataclass
class Properties:
    """A class to store and expand properties of modular robots easily."""

    color: Color  # The modules color.
    num_children: int  # The amount of possible children.


@dataclass
class BrickProperties(Properties):
    """A class to store and expand properties of bricks easily."""

    BOUNDING_BOX: Vector3  # The Bounding Box of the Brick (m).
    CHILD_OFFSET: float  # The attachment offset for the next child (m).
    MASS: float  # The Bricks Mass (kg).


@dataclass
class CoreProperties(Properties):
    """A class to store and expand properties of cores easily."""

    BOUNDING_BOX: Vector3  # The Bounding Box of the Core (m).
    CHILD_OFFSET: float  # The attachment offset for the next child (m).
    MASS: float  # The Cores Mass (kg).
    HORIZONTAL_OFFSET: float
    VERTICAL_OFFSET: float


@dataclass
class ActiveHingeProperties(Properties):
    """A class to store and expand properties of active hinges easily."""

    RANGE: float  # angle range of servo.
    EFFORT: float  # max effort of servo.
    VELOCITY: float  # max velocity of servo.
    FRAME_BOUNDING_BOX: Vector3
    FRAME_OFFSET: float
    SERVO1_BOUNDING_BOX: Vector3
    SERVO2_BOUNDING_BOX: Vector3

    FRAME_MASS: float  # The Mass of the Frame (kg).
    SERVO1_MASS: float  # The Mass of Servo1 (kg).
    SERVO2_MASS: float  # The Mass of Servo2 (kg).

    SERVO_OFFSET: float
    JOINT_OFFSET: float


@dataclass
class PropertySet:
    """A class to store a set of properties."""

    core_properties: CoreProperties
    brick_properties: BrickProperties
    active_hinge_properties: ActiveHingeProperties

    def get_core_properties(self) -> CoreProperties:
        """
        Get the cores properties.

        :returns: The cores properties.
        """
        return self.core_properties

    def get_brick_properties(self) -> BrickProperties:
        """
        Get the bricks properties.

        :returns: The brick properties.
        """
        return self.brick_properties

    def get_active_hinge_properties(self) -> ActiveHingeProperties:
        """
        Get the active hinge properties.

        :returns: The active hinges properties.
        """
        return self.active_hinge_properties
