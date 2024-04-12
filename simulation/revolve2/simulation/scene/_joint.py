import uuid
from dataclasses import dataclass, field

from ._pose import Pose
from ._rigid_body import RigidBody


@dataclass(kw_only=True)
class Joint:
    """Base class for all joints."""

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

    rigid_body1: RigidBody
    """The first attached rigid body."""

    rigid_body2: RigidBody
    """The second attached rigid body."""
