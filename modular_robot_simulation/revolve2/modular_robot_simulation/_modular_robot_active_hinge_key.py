from __future__ import annotations

from revolve2.modular_robot.body import ActiveHinge


class ModularRobotActiveHingeKey:
    """A wrapper around a modular robot active hinge that provides equality and hash function based on only the active hinge's id."""

    _value: ActiveHinge
    """The wrapper value."""

    def __init__(self, value: ActiveHinge) -> None:
        """
        Initialize this object.

        :param value: The value to wrap.
        """
        self._value = value

    @property
    def value(self) -> ActiveHinge:
        """
        Get the wrapper value.

        :returns: The wrapped value.
        """
        return self._value

    def __eq__(self, other: object) -> bool:
        """
        Compare with another ModularRobotActiveHingeKey using their ids.

        :param other: The object to compare with. Must be a ModularRobotActiveHingeKey.
        :returns: Whether their ids are equal.
        :raises ValueError: If comparing to anything other than a ModularRobotActiveHingeKey, or when the id of this or the other object has not been set.
        """
        if not isinstance(other, ModularRobotActiveHingeKey):
            raise ValueError(
                "Cannot compare with anything else than an object of type ModularRobotActiveHingeKey."
            )
        if self._value.id is None or other._value.id is None:
            raise ValueError(
                "Ids of both objects must be set. This probably means they are not part of a body."
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
                "Id of object must be set. This probably means it is not part of a body."
            )
        return hash(self._value.id)
