from __future__ import annotations

from revolve2.simulation.scene import JointHinge


class JointHingeKey:
    """A wrapper around a hinge joint that provides equality and hash function based on only its and its parent multi-body system's id."""

    _value: JointHinge
    """The wrapper value."""

    def __init__(self, value: JointHinge) -> None:
        """
        Initialize this object.

        :param value: The value to wrap.
        """
        self._value = value

    @property
    def value(self) -> JointHinge:
        """
        Get the wrapper value.

        :returns: The wrapped value.
        """
        return self._value

    def __eq__(self, other: object) -> bool:
        """
        Compare with another JointHingeKey using their ids.

        :param other: The object to compare with. Must be a JointHingeKey.
        :returns: Whether their ids are equal.
        :raises ValueError: If comparing to anything other than a JointHingeKey, or when the parent of this or the other object, or their wrapped values has not been set.
        """
        if not isinstance(other, JointHingeKey):
            raise ValueError(
                "Cannot compare with anything else than an object of type JointHingeKey."
            )
        if not self._value.has_parent_info or not other._value.has_parent_info:
            raise ValueError(
                "Parent information of both objects must be set. This probably means they are not part of a multi-body system."
            )
        if (
            not self._value.parent.has_parent_info
            or not other._value.parent.has_parent_info
        ):
            raise ValueError(
                "Parent information of parent multi-body systems not set. This probably means the multi-body system is not part of a scene."
            )
        return (
            self._value.parent.id == other._value.parent.id
            and self._value.id == other._value.id
        )

    def __hash__(self) -> int:
        """
        Hash this object using id only.

        :returns: The hash.
        :raises ValueError: When the id of this object has not been set.
        """
        if not self._value.has_parent_info:
            raise ValueError(
                "Parent information of object must be set. This probably means it is not part of a multi-body system."
            )
        if not self._value.parent.has_parent_info:
            raise ValueError(
                "Parent information of parent multi-body system not set. This probably means the multi-body system is not part of a scene."
            )
        return hash((self._value.parent.id, self._value.id))
