from __future__ import annotations

from revolve2.simulation.scene import MultiBodySystem


class MultiBodySystemKey:
    """A wrapper around a multi-body system that provides equality and hash function based on only its id."""

    _value: MultiBodySystem
    """The wrapper value."""

    def __init__(self, value: MultiBodySystem) -> None:
        """
        Initialize this object.

        :param value: The value to wrap.
        """
        self._value = value

    @property
    def value(self) -> MultiBodySystem:
        """
        Get the wrapper value.

        :returns: The wrapped value.
        """
        return self._value

    def __eq__(self, other: object) -> bool:
        """
        Compare with another MultiBodySystemKey using their ids.

        :param other: The object to compare with. Must be a MultiBodySystemKey.
        :returns: Whether their ids are equal.
        :raises ValueError: If comparing to anything other than a MultiBodySystemKey, or when the id of this or the other object has not been set.
        """
        if not isinstance(other, MultiBodySystemKey):
            raise ValueError(
                "Cannot compare with anything else than an object of type MultiBodySystemKey."
            )
        if self._value.id is None or other._value.id is None:
            raise ValueError(
                "Ids of both objects must be set. This probably means they are not part of a scene."
            )
        return self._value.id == other._value.id

    def __hash__(self) -> int:
        """
        Hash this object using id only.

        :returns: The hash.
        :raises ValueError: When the id of this object has not been set.
        """
        if self._value.id is None:
            raise ValueError(
                "Id of object must be set. This probably means it is not part of a scene."
            )
        return hash(self._value.id)
