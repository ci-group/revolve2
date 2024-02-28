from dataclasses import dataclass

from .._pose import Pose
from ._sensor import Sensor


@dataclass
class CameraSensor(Sensor):
    """Camera sensor."""

    pose: Pose
    """Pose of the geometry, relative to its parent rigid body."""
