from typing import Protocol, TypeVar

TSupportsLt = TypeVar("TSupportsLt", bound="SupportsLt")


class SupportsLt(Protocol):
    """Interface for types supporting the < operator."""

    def __lt__(self: TSupportsLt, other: TSupportsLt) -> bool:
        """
        Compare two objects using the < operator.

        :param other: The object to compare this with.
        """
        pass
