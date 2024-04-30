from __future__ import annotations

from enum import Enum, auto


class ViewerType(Enum):
    """Viewer types available for mujoco."""

    NATIVE = auto()
    CUSTOM = auto()

    @staticmethod
    def from_string(value: str) -> ViewerType:
        """
        Get viewer type from string.

        :param value: The value.
        :returns: The viewer type.
        :raises ValueError: If the passed value has no viewer type defined.
        """
        match value.lower():
            case "native":
                return ViewerType.NATIVE
            case "custom":
                return ViewerType.CUSTOM
            case _:
                raise ValueError(f"No viewer type {value} defined.")
