from dataclasses import dataclass, field

from pyrr import Quaternion, Vector3


@dataclass
class Pose:
    """A position and orientation."""

    position: Vector3 = field(default_factory=Vector3)
    """Position of the object."""

    orientation: Quaternion = field(default_factory=Quaternion)
    """Orientation of the object."""
