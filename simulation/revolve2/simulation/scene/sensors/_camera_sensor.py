from dataclasses import dataclass, field

from .._pose import Pose
from ._sensor import Sensor


@dataclass
class CameraSensor(Sensor):
    """Camera sensor."""

    pose: Pose
    camera_size: tuple[int, int]
    """Pose of the geometry, relative to its parent rigid body."""
    type: str = field(default="camera")
