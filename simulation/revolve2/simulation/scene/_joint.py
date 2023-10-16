from __future__ import annotations

import uuid
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from ._pose import Pose
from ._rigid_body import RigidBody

if TYPE_CHECKING:
    from ._multi_body_system import MultiBodySystem


@dataclass(kw_only=True)
class Joint:
    """Base class for all joints."""

    _uuid: uuid.UUID = field(init=False, default_factory=uuid.uuid1)

    @dataclass
    class _ParentInfo:
        parent: MultiBodySystem
        unique_id: int
        """Unique id within multi-body system."""

    _parent_info: _ParentInfo | None = field(default=None, init=False)

    @property
    def has_parent_info(self) -> bool:
        """
        Check whether parent information has been set.

        :returns: Whether parent information has been set.
        """
        return self._parent_info is not None

    @property
    def id(self) -> int:
        """
        Get the unique id of this object within its parent scene.

        :returns: The id
        :raises RuntimeError: If object does not have parent info.
        """
        if self._parent_info is None:
            raise RuntimeError("Object does not have parent info set.")
        return self._parent_info.unique_id

    @property
    def uuid(self) -> uuid.UUID:
        """
        Get the uuid.

        :returns: The uuid.
        """
        return self._uuid

    @property
    def parent(self) -> MultiBodySystem:
        """
        Get the parent multi-body system of this object.

        :returns: The parent.
        :raises RuntimeError: If object does not have parent info.
        """
        if self._parent_info is None:
            raise RuntimeError("Object does not have parent info set.")
        return self._parent_info.parent

    pose: Pose
    """Pose of the joint."""

    rigid_body1: RigidBody
    """The first attached rigid body."""

    rigid_body2: RigidBody
    """The second attached rigid body."""
