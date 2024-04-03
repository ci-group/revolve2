import uuid
from dataclasses import dataclass, field
from pyrr import Vector3

from ._pose import Pose
from ._rigid_body import RigidBody


@dataclass(kw_only=True)
class Motor:
    """Base class for all motors."""

    _uuid: uuid.UUID = field(init=False, default_factory=uuid.uuid1)

    @property
    def uuid(self) -> uuid.UUID:
        """
        Get the uuid.

        :returns: The uuid.
        """
        return self._uuid

    pose: Pose
    """Pose of the joint."""

    site: str 
    """Site name of motor."""

    axis: Vector3
    """Directional vector the joint rotates about."""

    ctrlrange: list
    """The upper and lower range of the motor. e.g. [0,100]"""

    gear: float
    """Gear of the motor"""
