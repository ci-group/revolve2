from typing import Generic, Protocol, TypeVar
from uuid import UUID


class HasUUID(Protocol):
    """A class where each instance has a UUID."""

    @property
    def uuid(self) -> UUID:
        """
        Get the uuid.

        :returns: The uuid.

        # noqa: DAR202
        """
        pass


_T = TypeVar("_T", bound=HasUUID)


class UUIDKey(Generic[_T]):
    """Wraps a value and implements __eq__ and __hash__ based purely on id(value)."""

    _value: _T

    def __init__(self, value: _T) -> None:
        """
        Initialize this object.

        :param value: The value to wrap.
        """
        self._value = value

    @property
    def value(self) -> _T:
        """
        Get the wrapped value.

        :returns: The value.
        """
        return self._value

    def __eq__(self, other: object) -> bool:
        """
        Compare with another wrapped value using their ids.

        :param other: The object to compare with.
        :returns: Whether their ids are equal.
        :raises ValueError: If the other objecgt is not an UUIDKey.
        """
        if not isinstance(other, UUIDKey) or not isinstance(
            other._value, type(self._value)
        ):
            raise ValueError()
        return self._value.uuid == other._value.uuid

    def __hash__(self) -> int:
        """
        Hash this object using its id only.

        :returns: The hash.
        """
        return hash(self._value.uuid)
