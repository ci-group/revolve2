from pyrr import Vector3
from revolve2.modular_robot._properties import (
    ActiveHingeProperties,
    BrickProperties,
    CoreProperties,
    PropertySet,
)
from revolve2.simulation.actor import Color

core_properties = CoreProperties(
    BOUNDING_BOX=Vector3([0.15, 0.15, 0.15]),
    MASS=0.250,
    CHILD_OFFSET=0.15 / 2.0,
    color=Color(255, 50, 50, 255),
    num_children=4,
    HORIZONTAL_OFFSET=0.029,
    VERTICAL_OFFSET=0.032,
)

brick_properties = BrickProperties(
    BOUNDING_BOX=Vector3([0.06288625, 0.06288625, 0.0603]),
    MASS=0.030,
    CHILD_OFFSET=0.06288625 / 2.0,
    color=Color(50, 50, 255, 255),
    num_children=3,
)

active_hinge_properties = ActiveHingeProperties(
    RANGE=1.047197551,
    EFFORT=0.948013269,
    VELOCITY=6.338968228,
    color=Color(255, 255, 255, 255),
    num_children=1,
    FRAME_BOUNDING_BOX=Vector3([0.018, 0.053, 0.0165891]),
    FRAME_OFFSET=0.04525,
    SERVO1_BOUNDING_BOX=Vector3([0.0583, 0.0512, 0.020]),
    SERVO2_BOUNDING_BOX=Vector3([0.002, 0.053, 0.053]),
    FRAME_MASS=0.011,
    SERVO1_MASS=0.058,
    SERVO2_MASS=0.02,
    SERVO_OFFSET=0.0299,
    JOINT_OFFSET=0.0119,
)


class V2PropertySet(PropertySet):
    """The Property Set for V2 Robots."""

    def __init__(self) -> None:
        """Initialize the Object."""
        super().__init__(core_properties, brick_properties, active_hinge_properties)
