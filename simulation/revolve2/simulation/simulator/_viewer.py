from abc import ABC, abstractmethod
from typing import Any


class Viewer(ABC):
    """An abstract viewer class, enabling the rendering of simulations."""

    @abstractmethod
    def close_viewer(self) -> None:
        """Close the viewer."""

    @abstractmethod
    def render(self) -> Any | None:
        """
        Render the scene on the viewer.

        :returns: Nothing or feedback.
        """
        raise NotImplementedError

    @abstractmethod
    def current_viewport_size(self) -> tuple[int, int]:
        """
        Get the current viewport size.

        :returns: The size as a tuple.
        """

    @property
    @abstractmethod
    def view_port(self) -> Any:
        """
        Get the viewport.

        :returns: The viewport object.
        """

    @property
    @abstractmethod
    def context(self) -> Any:
        """
        Return the viewer context.

        :returns: The context object.
        """

    @property
    @abstractmethod
    def can_record(self) -> bool:
        """
        Check if this viewer can record.

        :returns: The truth.
        """
